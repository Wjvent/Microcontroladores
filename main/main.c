/**
 * @file main.c
 * @brief Portón automático con ESP32 + ESP-IDF + MQTT
 * @version 1.0
 *
 * Cambios: Portal WiFi/MQTT con NVS (sin defaults), formularios separados (act=wifi / act=mqtt),
 * wipe para volver a AP, timeout 30s sin IP -> reinicia a AP, AP se apaga al conectar STA.
 * Se agregó decoder URL (url_decode_inplace) y soporte POST para evitar 431 Header too large.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "cJSON.h"
#include "esp_http_server.h"

// ------------------------------ ESTADOS ---------------------------------------
#define ESTADO_INICIAL     0
#define ESTADO_ERROR       1
#define ESTADO_ABRIENDO    2
#define ESTADO_ABIERTO     3
#define ESTADO_CERRANDO    4
#define ESTADO_CERRADO     5
#define ESTADO_DETENIDO    6
#define ESTADO_DESCONOCIDO 7

#define ERR_OK                 0
#define ERR_TIMEOUT_OPEN       1
#define ERR_TIMEOUT_CLOSE      2
#define ERR_LS_INCONSISTENT    3
#define ERR_STATE_GUARDRAIL   99

// ----------------------- CONFIGURACIÓN AJUSTABLE ------------------------------
#define PIN_LSC        GPIO_NUM_35
#define PIN_LSA        GPIO_NUM_34
#define PIN_MOTOR_A    GPIO_NUM_13
#define PIN_MOTOR_C    GPIO_NUM_12
#define PIN_LAMP       GPIO_NUM_2

#define LM_ACTIVO      0
#define LM_NOACTIVO    (!LM_ACTIVO)

#define T_OPEN_MS      15000
#define T_CLOSE_MS     15000
#define DEBOUNCE_MS    20
#define PUB_PERIOD_MS  30000

// ===================== Portal WiFi AP/STA + MQTT (sin defaults) =====================
#define AP_SSID      "ESP_CONFIG_AP"
#define AP_PASS      "12345678"
#define AP_MAX_CONN  4

// NVS keys
#define NVS_NAMESPACE       "config"
#define NVS_KEY_WIFI_SSID   "wifi_ssid"
#define NVS_KEY_WIFI_PASS   "wifi_pass"
#define NVS_KEY_BOOTMODE    "boot_mode"
#define NVS_KEY_MQTT_URI    "mqtt_uri"
#define NVS_KEY_TOPIC1      "topic1"   // CMD (suscripción)
#define NVS_KEY_TOPIC2      "topic2"   // STATUS (publicación)
#define NVS_KEY_TOPIC3      "topic3"   // TELE (publicación)

#define BOOTMODE_CONFIG_AP  0
#define BOOTMODE_STA_ONLY   1

static const char *TAG = "GATE";
static esp_mqtt_client_handle_t g_client = NULL;

// WiFi (variables configurables, SIN defaults)
static char g_wifi_ssid_cfg[33] = "";
static char g_wifi_pass_cfg[65] = "";
static bool g_have_creds = false;
static bool g_wifi_connected = false;
static char g_sta_ip[16] = "0.0.0.0";
static char g_status_msg[160] = "Ingrese SSID, contrasena y parametros MQTT; luego Guardar.";
static uint8_t g_boot_mode = BOOTMODE_CONFIG_AP;
static bool g_ap_enabled = false;

// MQTT (variables configurables, SIN defaults)
static char g_mqtt_uri[128]    = "";   // p.ej. mqtt://host:1883
static char g_topic_cmd[96]    = "";   // suscripción
static char g_topic_status[96] = "";   // publicación estado
static char g_topic_tele[96]   = "";   // publicación tele

// FSM/cola
typedef enum {
    CMD_NONE = 0,
    CMD_OPEN,
    CMD_CLOSE,
    CMD_STOP,
    CMD_TOGGLE,
    CMD_LAMP_ON,
    CMD_LAMP_OFF
} gate_cmd_t;

static QueueHandle_t q_cmd;
static uint64_t g_last_pub_us = 0;
static volatile int g_estado = ESTADO_INICIAL;
static int g_estado_prev = -1;
static int g_motorA = 0, g_motorC = 0;
static int g_lsa = 0, g_lsc = 0;
static int g_error_code = ERR_OK;

static httpd_handle_t g_httpd = NULL;

// Timeout conexión
static TickType_t g_connect_start_tick = 0;
static bool g_connect_timer_active = false;

// ---------- Prototipos ----------
static void mqtt_init(void);
static void mqtt_restart(void);
static inline bool fetch_cmd(gate_cmd_t *out_cmd);
static inline void tick_telemetria(void);
static httpd_handle_t start_webserver(void);
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t root_post_handler(httpd_req_t *req);

// ---------- URL decode helper ----------
static inline int hex2val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    c |= 32; // tolower
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
}

/**
 * @brief Decodifica en sitio una cadena application/x-www-form-urlencoded.
 *  - Reemplaza '+' por espacio
 *  - Decodifica %HH hexadecimal
 *  - Garantiza terminación '\0'
 */
static void url_decode_inplace(char *s) {
    if (!s) return;
    char *r = s, *w = s;
    while (*r) {
        char ch = *r;
        if (ch == '+') { *w++ = ' '; r++; }
        else if (ch == '%' && r[1] && r[2]) {
            int hi = hex2val(r[1]);
            int lo = hex2val(r[2]);
            if (hi >= 0 && lo >= 0) {
                *w++ = (char)((hi << 4) | lo);
                r += 3;
            } else {
                // Secuencia inválida, copia literal '%'
                *w++ = *r++;
            }
        } else {
            *w++ = *r++;
        }
    }
    *w = '\0';
}

// ---------- NVS helpers ----------
static void save_boot_mode_to_nvs(uint8_t mode) {
    nvs_handle_t h; if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, NVS_KEY_BOOTMODE, mode); nvs_commit(h); nvs_close(h);
    ESP_LOGI(TAG, "Boot mode -> %u", mode);
}
static void load_boot_mode_from_nvs(void) {
    nvs_handle_t h; if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) { g_boot_mode = BOOTMODE_CONFIG_AP; return; }
    uint8_t m; if (nvs_get_u8(h, NVS_KEY_BOOTMODE, &m) == ESP_OK) g_boot_mode = m; else g_boot_mode = BOOTMODE_CONFIG_AP;
    nvs_close(h); ESP_LOGI(TAG, "Boot mode NVS = %u", g_boot_mode);
}
static void save_wifi_creds_to_nvs(void) {
    nvs_handle_t h; if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, NVS_KEY_WIFI_SSID, g_wifi_ssid_cfg);
    nvs_set_str(h, NVS_KEY_WIFI_PASS, g_wifi_pass_cfg);
    nvs_commit(h); nvs_close(h); ESP_LOGI(TAG, "WiFi creds guardadas en NVS");
}
static void load_wifi_creds_from_nvs(void) {
    nvs_handle_t h; if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) { g_have_creds=false; return; }
    size_t len=sizeof(g_wifi_ssid_cfg); if (nvs_get_str(h, NVS_KEY_WIFI_SSID, g_wifi_ssid_cfg, &len) != ESP_OK) g_wifi_ssid_cfg[0]=0;
    len=sizeof(g_wifi_pass_cfg); if (nvs_get_str(h, NVS_KEY_WIFI_PASS, g_wifi_pass_cfg, &len) != ESP_OK) g_wifi_pass_cfg[0]=0;
    nvs_close(h); g_have_creds = (g_wifi_ssid_cfg[0] != 0);
}
static void save_mqtt_to_nvs(void) {
    nvs_handle_t h; if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, NVS_KEY_MQTT_URI, g_mqtt_uri);
    nvs_set_str(h, NVS_KEY_TOPIC1,   g_topic_cmd);
    nvs_set_str(h, NVS_KEY_TOPIC2,   g_topic_status);
    nvs_set_str(h, NVS_KEY_TOPIC3,   g_topic_tele);
    nvs_commit(h); nvs_close(h); ESP_LOGI(TAG, "MQTT (URI y topicos) guardados en NVS");
}
static void load_mqtt_from_nvs(void) {
    nvs_handle_t h; if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    size_t len;
    len=sizeof(g_mqtt_uri);    nvs_get_str(h, NVS_KEY_MQTT_URI, g_mqtt_uri, &len);
    len=sizeof(g_topic_cmd);   nvs_get_str(h, NVS_KEY_TOPIC1,   g_topic_cmd, &len);
    len=sizeof(g_topic_status);nvs_get_str(h, NVS_KEY_TOPIC2,   g_topic_status, &len);
    len=sizeof(g_topic_tele);  nvs_get_str(h, NVS_KEY_TOPIC3,   g_topic_tele, &len);
    nvs_close(h);
}
static void erase_all_creds_nvs(void) {
    nvs_handle_t h; if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, NVS_KEY_WIFI_SSID);
    nvs_erase_key(h, NVS_KEY_WIFI_PASS);
    nvs_erase_key(h, NVS_KEY_MQTT_URI);
    nvs_erase_key(h, NVS_KEY_TOPIC1);
    nvs_erase_key(h, NVS_KEY_TOPIC2);
    nvs_erase_key(h, NVS_KEY_TOPIC3);
    nvs_set_u8(h, NVS_KEY_BOOTMODE, BOOTMODE_CONFIG_AP);
    nvs_commit(h); nvs_close(h);
    g_wifi_ssid_cfg[0]=0; g_wifi_pass_cfg[0]=0; g_have_creds=false;
    g_mqtt_uri[0]=0; g_topic_cmd[0]=0; g_topic_status[0]=0; g_topic_tele[0]=0;
    ESP_LOGW(TAG, "Credenciales WiFi/MQTT borradas de NVS.");
}

// ---------- Apl. de parámetros (reutilizable para GET y POST) ----------
static void apply_wifi_from_kvstring(const char *kv) {
    char ssid[64]={0}, pass[64]={0};

    if (httpd_query_key_value(kv, "ssid", ssid, sizeof(ssid)) == ESP_OK) {
        url_decode_inplace(ssid);
        // pass puede ser vacío
        httpd_query_key_value(kv, "pass", pass, sizeof(pass));
        url_decode_inplace(pass);

        if (ssid[0] == '\0') {
            snprintf(g_status_msg, sizeof(g_status_msg), "SSID vacio. Ingrese un SSID valido.");
        } else {
            strncpy(g_wifi_ssid_cfg, ssid, sizeof(g_wifi_ssid_cfg)-1);
            strncpy(g_wifi_pass_cfg, pass, sizeof(g_wifi_pass_cfg)-1);
            g_have_creds = true;
            save_wifi_creds_to_nvs();

            wifi_config_t sta_cfg = (wifi_config_t){0};
            strncpy((char*)sta_cfg.sta.ssid, g_wifi_ssid_cfg, sizeof(sta_cfg.sta.ssid));
            strncpy((char*)sta_cfg.sta.password, g_wifi_pass_cfg, sizeof(sta_cfg.sta.password));
            sta_cfg.sta.threshold.authmode = strlen(g_wifi_pass_cfg) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
            esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);

            snprintf(g_status_msg, sizeof(g_status_msg), "Guardado WiFi. Conectando a '%s'...", g_wifi_ssid_cfg);
            esp_wifi_disconnect();
            esp_wifi_connect();
            g_connect_start_tick = xTaskGetTickCount();
            g_connect_timer_active = true;
            save_boot_mode_to_nvs(BOOTMODE_CONFIG_AP); // Mantener AP hasta obtener IP
        }
    } else {
        snprintf(g_status_msg, sizeof(g_status_msg), "Falta el parametro SSID.");
    }
}

static void apply_mqtt_from_kvstring(const char *kv) {
    char broker[128]={0}, t1[96]={0}, t2[96]={0}, t3[96]={0};

    if (httpd_query_key_value(kv, "broker", broker, sizeof(broker)) == ESP_OK) {
        url_decode_inplace(broker);
        if (broker[0]) strncpy(g_mqtt_uri, broker, sizeof(g_mqtt_uri)-1);
    }
    if (httpd_query_key_value(kv, "t1", t1, sizeof(t1)) == ESP_OK) {
        url_decode_inplace(t1);
        if (t1[0]) strncpy(g_topic_cmd, t1, sizeof(g_topic_cmd)-1);
    }
    if (httpd_query_key_value(kv, "t2", t2, sizeof(t2)) == ESP_OK) {
        url_decode_inplace(t2);
        if (t2[0]) strncpy(g_topic_status, t2, sizeof(g_topic_status)-1);
    }
    if (httpd_query_key_value(kv, "t3", t3, sizeof(t3)) == ESP_OK) {
        url_decode_inplace(t3);
        if (t3[0]) strncpy(g_topic_tele, t3, sizeof(g_topic_tele)-1);
    }

    save_mqtt_to_nvs();
    mqtt_restart();  // NO toca WiFi
    snprintf(g_status_msg, sizeof(g_status_msg), "Parametros MQTT actualizados.");
}

// ---------- HTTP portal (GET) ----------
static esp_err_t root_get_handler(httpd_req_t *req) {
    char query[512]; int qlen = httpd_req_get_url_query_len(req);

    if (qlen > 0 && qlen < (int)sizeof(query)) {
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {

            // 1) BORRAR CREDENCIALES
            char wipe[8]={0};
            if (httpd_query_key_value(query, "wipe", wipe, sizeof(wipe)) == ESP_OK && !strcmp(wipe,"1")) {
                erase_all_creds_nvs();
                esp_wifi_stop();
                esp_wifi_restore();          // limpia config del driver
                save_boot_mode_to_nvs(BOOTMODE_CONFIG_AP);
                httpd_resp_set_type(req, "text/html");
                httpd_resp_sendstr_chunk(req, "<html><body><h3>Credenciales borradas.</h3><p>Reiniciando...</p></body></html>");
                httpd_resp_sendstr_chunk(req, NULL);
                vTaskDelay(pdMS_TO_TICKS(250));
                esp_restart();
                return ESP_OK;
            }

            // 2) ACCION: act=wifi | act=mqtt (acepta también por GET si es corto)
            char act[16]={0};
            if (httpd_query_key_value(query, "act", act, sizeof(act)) == ESP_OK) {
                if (!strcmp(act,"wifi")) {
                    apply_wifi_from_kvstring(query);
                } else if (!strcmp(act,"mqtt")) {
                    apply_mqtt_from_kvstring(query);
                }
            }
        }
    }

    // ------------------- HTML (chunked) -------------------
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Config ESP32</title></head><body>");
    httpd_resp_sendstr_chunk(req, "<h2>Porton Automatico</h2>");

    char buf[512];
    snprintf(buf, sizeof(buf), "<p><b>Mensaje:</b> %s</p>", g_status_msg); httpd_resp_sendstr_chunk(req, buf);

    const char *connected = g_wifi_connected ? "SI" : "NO";
    httpd_resp_sendstr_chunk(req, "<hr><h3>WiFi (STA)</h3>");
    snprintf(buf, sizeof(buf), "<p>SSID actual: %s</p>", g_wifi_ssid_cfg[0] ? g_wifi_ssid_cfg : "(no configurado)"); httpd_resp_sendstr_chunk(req, buf);
    snprintf(buf, sizeof(buf), "<p>Conectado: %s</p>", connected); httpd_resp_sendstr_chunk(req, buf);
    snprintf(buf, sizeof(buf), "<p>IP STA: %s</p>", g_wifi_connected ? g_sta_ip : "0.0.0.0"); httpd_resp_sendstr_chunk(req, buf);

    // -------- FORM SOLO WIFI (act=wifi) -> POST --------
    httpd_resp_sendstr_chunk(req, "<form action='/' method='POST'>");
    httpd_resp_sendstr_chunk(req, "<input type='hidden' name='act' value='wifi'>");
    httpd_resp_sendstr_chunk(req, "<fieldset><legend>Red WiFi</legend>");
    snprintf(buf, sizeof(buf), "SSID: <input name='ssid' value='%s' required><br><br>", g_wifi_ssid_cfg); httpd_resp_sendstr_chunk(req, buf);
    httpd_resp_sendstr_chunk(req, "Password: <input type='password' name='pass'><br>");
    httpd_resp_sendstr_chunk(req, "</fieldset><br>");
    httpd_resp_sendstr_chunk(req, "<button type='submit'>Guardar WiFi</button>");
    httpd_resp_sendstr_chunk(req, "</form>");

    // -------- FORM SOLO MQTT (act=mqtt) -> POST --------
    httpd_resp_sendstr_chunk(req, "<br><form action='/' method='POST'>");
    httpd_resp_sendstr_chunk(req, "<input type='hidden' name='act' value='mqtt'>");
    httpd_resp_sendstr_chunk(req, "<fieldset><legend>MQTT</legend>");
    snprintf(buf, sizeof(buf), "Broker (URI): <input name='broker' value='%s' placeholder='mqtt://host:1883' style='width:360px'><br><br>", g_mqtt_uri); httpd_resp_sendstr_chunk(req, buf);
    snprintf(buf, sizeof(buf), "Topico 1 (CMD - suscripcion): <input name='t1' value='%s' style='width:360px'><br><br>", g_topic_cmd); httpd_resp_sendstr_chunk(req, buf);
    snprintf(buf, sizeof(buf), "Topico 2 (STATUS - publicacion): <input name='t2' value='%s' style='width:360px'><br><br>", g_topic_status); httpd_resp_sendstr_chunk(req, buf);
    snprintf(buf, sizeof(buf), "Topico 3 (TELE - publicacion): <input name='t3' value='%s' style='width:360px'><br>", g_topic_tele); httpd_resp_sendstr_chunk(req, buf);
    httpd_resp_sendstr_chunk(req, "</fieldset><br>");
    httpd_resp_sendstr_chunk(req, "<button type='submit'>Guardar MQTT</button>");
    httpd_resp_sendstr_chunk(req, "</form>");

    // -------- BOTON BORRAR --------
    httpd_resp_sendstr_chunk(req, "<hr><form action='/' method='GET'>");
    httpd_resp_sendstr_chunk(req, "<input type='hidden' name='wipe' value='1'>");
    httpd_resp_sendstr_chunk(req, "<button type='submit' style='background:#c00;color:#fff;padding:8px 12px;border:0;border-radius:6px;'>Borrar credenciales y volver a AP</button>");
    httpd_resp_sendstr_chunk(req, "</form>");

    snprintf(buf, sizeof(buf), "<p>AP de configuracion: SSID '%s' / pass '%s' (activo solo si no hay conexion).</p>", AP_SSID, AP_PASS); httpd_resp_sendstr_chunk(req, buf);
    httpd_resp_sendstr_chunk(req, "</body></html>");
    httpd_resp_sendstr_chunk(req, NULL);

    return ESP_OK;
}

// ---------- HTTP portal (POST) ----------
static esp_err_t root_post_handler(httpd_req_t *req) {
    // Leemos el body (application/x-www-form-urlencoded)
    int total = req->content_len;
    if (total <= 0 || total > 2048) { // límite prudente
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too long or empty");
        return ESP_OK;
    }

    char *body = malloc(total + 1);
    if (!body) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No mem"); return ESP_OK; }

    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) { free(body); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv error"); return ESP_OK; }
        received += r;
    }
    body[total] = '\0';

    char act[16]={0};
    if (httpd_query_key_value(body, "act", act, sizeof(act)) == ESP_OK) {
        if (!strcmp(act, "wifi")) {
            apply_wifi_from_kvstring(body);
        } else if (!strcmp(act, "mqtt")) {
            apply_mqtt_from_kvstring(body);
        }
    }

    // Redirigir a '/'
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);

    free(body);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_get = { .uri = "/", .method = HTTP_GET,  .handler = root_get_handler,  .user_ctx = NULL };
        httpd_uri_t root_post= { .uri = "/", .method = HTTP_POST, .handler = root_post_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &root_get);
        httpd_register_uri_handler(server, &root_post);
        ESP_LOGI(TAG, "HTTP server en puerto %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "No se pudo iniciar HTTP server");
    }
    return server;
}

// ---------- Eventos WiFi/IP ----------
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            if (g_have_creds) {
                snprintf(g_status_msg, sizeof(g_status_msg), "Intentando conectar a '%s'...", g_wifi_ssid_cfg);
                esp_wifi_connect();
                g_connect_start_tick = xTaskGetTickCount();
                g_connect_timer_active = true;
            }
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)data;
            g_wifi_connected = false;
            snprintf(g_status_msg, sizeof(g_status_msg), "Desconectado (razon %d). Reintentando...", disc->reason);
            if (g_have_creds) esp_wifi_connect();
            break;
        }
        default: break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(g_sta_ip, sizeof(g_sta_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        g_wifi_connected = true;
        g_connect_timer_active = false;
        snprintf(g_status_msg, sizeof(g_status_msg), "Conectado a '%s'. IP: %s", g_wifi_ssid_cfg, g_sta_ip);

        if (g_ap_enabled) { esp_wifi_set_mode(WIFI_MODE_STA); g_ap_enabled = false; }
        save_boot_mode_to_nvs(BOOTMODE_STA_ONLY);
    }
}

// ---------- Timeout 30s ----------
static void connect_timeout_task(void *arg) {
    const TickType_t TO = pdMS_TO_TICKS(30000);
    while (1) {
        if (g_connect_timer_active && !g_wifi_connected) {
            if ((xTaskGetTickCount() - g_connect_start_tick) > TO) {
                ESP_LOGW(TAG, "Timeout 30s sin IP. Volviendo a modo configuracion...");
                save_boot_mode_to_nvs(BOOTMODE_CONFIG_AP);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------- Inicialización WiFi ----------
static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    load_wifi_creds_from_nvs();
    load_mqtt_from_nvs();
    load_boot_mode_from_nvs();

    esp_netif_create_default_wifi_sta();
    if (g_boot_mode == BOOTMODE_CONFIG_AP || !g_have_creds) esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // No guardar config WiFi del driver en FLASH
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    if (g_boot_mode == BOOTMODE_CONFIG_AP || !g_have_creds) {
        wifi_config_t ap_cfg = { 0 };
        strncpy((char*)ap_cfg.ap.ssid, AP_SSID, sizeof(ap_cfg.ap.ssid));
        ap_cfg.ap.ssid_len       = strlen(AP_SSID);
        strncpy((char*)ap_cfg.ap.password, AP_PASS, sizeof(ap_cfg.ap.password));
        ap_cfg.ap.channel        = 1;
        ap_cfg.ap.max_connection = AP_MAX_CONN;
        ap_cfg.ap.authmode       = strlen(AP_PASS) ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
        g_ap_enabled = true;
        ESP_ERROR_CHECK(esp_wifi_start());
        snprintf(g_status_msg, sizeof(g_status_msg), "Ingrese SSID, pass y parametros MQTT; luego Guardar.");
        ESP_LOGI(TAG, "AP de config: '%s' pass '%s' (http://192.168.4.1/)", AP_SSID, AP_PASS);
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        g_ap_enabled = false;
        ESP_ERROR_CHECK(esp_wifi_start());

        if (g_have_creds) {
            wifi_config_t sta_cfg = { 0 };
            strncpy((char*)sta_cfg.sta.ssid, g_wifi_ssid_cfg, sizeof(sta_cfg.sta.ssid));
            strncpy((char*)sta_cfg.sta.password, g_wifi_pass_cfg, sizeof(sta_cfg.sta.password));
            sta_cfg.sta.threshold.authmode = strlen(g_wifi_pass_cfg) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
            snprintf(g_status_msg, sizeof(g_status_msg), "Intentando conectar a '%s' (desde NVS)...", g_wifi_ssid_cfg);
            esp_wifi_connect();
            g_connect_start_tick = xTaskGetTickCount();
            g_connect_timer_active = true;
        }
    }

    g_httpd = start_webserver();
    xTaskCreate(connect_timeout_task, "connect_timeout_task", 4096, NULL, 5, NULL);
}

// ------------------------------ UTILIDADES / FSM ------------------------------
static int debounce_read(gpio_num_t pin, int ms) {
    const int step = 5;
    int stable = gpio_get_level(pin);
    int elapsed = 0;
    while (elapsed < ms) {
        vTaskDelay(pdMS_TO_TICKS(step));
        int now = gpio_get_level(pin);
        if (now != stable) { stable = now; elapsed = 0; } else { elapsed += step; }
    }
    return stable;
}
static inline void leer_sensores(void) {
    g_lsa = (debounce_read(PIN_LSA, DEBOUNCE_MS) == LM_ACTIVO);
    g_lsc = (debounce_read(PIN_LSC, DEBOUNCE_MS) == LM_ACTIVO);
}
static const char* estado_str(int e) {
    switch (e) {
        case ESTADO_INICIAL:     return "INICIAL";
        case ESTADO_ERROR:       return "ERROR";
        case ESTADO_ABRIENDO:    return "ABRIENDO";
        case ESTADO_ABIERTO:     return "ABIERTO";
        case ESTADO_CERRANDO:    return "CERRANDO";
        case ESTADO_CERRADO:     return "CERRADO";
        case ESTADO_DETENIDO:    return "DETENIDO";
        case ESTADO_DESCONOCIDO: return "DESCONOCIDO";
        default:                 return "???";
    }
}
static inline void motor_stop(void)   { gpio_set_level(PIN_MOTOR_A, 0); gpio_set_level(PIN_MOTOR_C, 0); g_motorA=g_motorC=0; }
static inline void motor_abrir(void)  { gpio_set_level(PIN_MOTOR_C, 0); vTaskDelay(pdMS_TO_TICKS(10)); gpio_set_level(PIN_MOTOR_A, 1); g_motorA=1; g_motorC=0; }
static inline void motor_cerrar(void) { gpio_set_level(PIN_MOTOR_A, 0); vTaskDelay(pdMS_TO_TICKS(10)); gpio_set_level(PIN_MOTOR_C, 1); g_motorA=0; g_motorC=1; }
static inline void lamp_on(bool on)   { gpio_set_level(PIN_LAMP, on ? 1 : 0); }

static void publicar_json(const char *topic, bool include_mot, bool include_err) {
    if (!g_client || !topic || !topic[0]) return;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state", estado_str(g_estado));
    cJSON_AddBoolToObject(root, "lsa_open", g_lsa);
    cJSON_AddBoolToObject(root, "lsc_closed", g_lsc);
    if (include_mot) {
        cJSON_AddBoolToObject(root, "motor_open", g_motorA);
        cJSON_AddBoolToObject(root, "motor_close", g_motorC);
    }
    if (include_err) cJSON_AddNumberToObject(root, "err", g_error_code);
    char *js = cJSON_PrintUnformatted(root);
    if (js) { esp_mqtt_client_publish(g_client, topic, js, 0, 1, 1); free(js); }
    cJSON_Delete(root);
}
static inline void publicar_estado_si_cambia(void) {
    if (g_estado != g_estado_prev) {
        g_estado_prev = g_estado;
        publicar_json(g_topic_status, true, true);
        ESP_LOGI(TAG, "Estado => %s", estado_str(g_estado));
    }
}
static inline void tick_telemetria(void) {
    uint64_t now = esp_timer_get_time();
    if ((now - g_last_pub_us) > (uint64_t)PUB_PERIOD_MS * 1000ULL) {
        publicar_json(g_topic_tele, true, true);
        g_last_pub_us = now;
    }
}
static inline bool fetch_cmd(gate_cmd_t *out_cmd) { return xQueueReceive(q_cmd, out_cmd, 0) == pdTRUE; }

// ------------------------------- MQTT dinámico -------------------------------
static gate_cmd_t parse_cmd_json(const char *data, int len) {
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root) return CMD_NONE;
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    gate_cmd_t out = CMD_NONE;
    if (cJSON_IsString(cmd) && cmd->valuestring) {
        if (!strcasecmp(cmd->valuestring, "OPEN")) out = CMD_OPEN;
        else if (!strcasecmp(cmd->valuestring, "CLOSE")) out = CMD_CLOSE;
        else if (!strcasecmp(cmd->valuestring, "STOP")) out = CMD_STOP;
        else if (!strcasecmp(cmd->valuestring, "TOGGLE")) out = CMD_TOGGLE;
        else if (!strcasecmp(cmd->valuestring, "LAMP_ON")) out = CMD_LAMP_ON;
        else if (!strcasecmp(cmd->valuestring, "LAMP_OFF")) out = CMD_LAMP_OFF;
    }
    cJSON_Delete(root);
    return out;
}
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t e = event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado (%s)", g_mqtt_uri);
            if (g_topic_cmd[0]) esp_mqtt_client_subscribe(g_client, g_topic_cmd, 1);
            publicar_json(g_topic_status, true, false);
            break;
        case MQTT_EVENT_DATA: {
            char *buf = calloc(1, e->data_len + 1); if (!buf) break;
            memcpy(buf, e->data, e->data_len);
            gate_cmd_t cmd = parse_cmd_json(buf, e->data_len);
            free(buf);
            if (cmd != CMD_NONE && q_cmd) xQueueSend(q_cmd, &cmd, 0);
            break;
        }
        default: break;
    }
}
static void mqtt_init(void) {
    if (!g_mqtt_uri[0]) {
        ESP_LOGW(TAG, "MQTT no iniciado: broker vacio.");
        return;
    }

    esp_mqtt_client_config_t cfg = {
        .broker  = { .address.uri = g_mqtt_uri },
        .session = { .keepalive = 30, .disable_clean_session = false },
    };

    g_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(g_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(g_client);
}

static void mqtt_restart(void) {
    if (g_client) { esp_mqtt_client_stop(g_client); esp_mqtt_client_destroy(g_client); g_client = NULL; }
    mqtt_init();
}

// --------------------------- BUCLES DE ESTADO --------------------------------
static int loop_error(void) {
    ESP_LOGW(TAG, "Entrando a ERROR (code=%d).", g_error_code);
    motor_stop(); publicar_estado_si_cambia();
    while (1) {
        leer_sensores();
        if (!(g_lsa && g_lsc)) {
            if (g_lsc && !g_lsa) return ESTADO_CERRADO;
            if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
            if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;
        }
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
            if (cmd == CMD_OPEN)     return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE)    return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE)   return ESTADO_ABRIENDO;
        }
        tick_telemetria(); publicar_estado_si_cambia(); vTaskDelay(pdMS_TO_TICKS(20));
    }
}
static int loop_abierto(void) {
    motor_stop(); publicar_estado_si_cambia();
    while (1) {
        leer_sensores();
        if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;
        if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_CLOSE || cmd == CMD_TOGGLE) return ESTADO_CERRANDO;
            if (cmd == CMD_STOP)   return ESTADO_DETENIDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }
        tick_telemetria(); publicar_estado_si_cambia(); vTaskDelay(pdMS_TO_TICKS(20));
    }
}
static int loop_cerrado(void) {
    motor_stop(); publicar_estado_si_cambia();
    while (1) {
        leer_sensores();
        if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (!g_lsa && !g_lsc) return ESTADO_DESCONOCIDO;
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN || cmd == CMD_TOGGLE) return ESTADO_ABRIENDO;
            if (cmd == CMD_STOP)   return ESTADO_DETENIDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }
        tick_telemetria(); publicar_estado_si_cambia(); vTaskDelay(pdMS_TO_TICKS(20));
    }
}
static int loop_detenido(void) {
    motor_stop(); publicar_estado_si_cambia();
    while (1) {
        leer_sensores();
        if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN)   return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE)  return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE) return g_lsc ? ESTADO_ABRIENDO : ESTADO_CERRANDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }
        tick_telemetria(); publicar_estado_si_cambia(); vTaskDelay(pdMS_TO_TICKS(20));
    }
}
static int loop_desconocido(void) {
    motor_stop(); publicar_estado_si_cambia();
    while (1) {
        leer_sensores();
        if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
        if (g_lsc && !g_lsa) return ESTADO_CERRADO;
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_OPEN)     return ESTADO_ABRIENDO;
            if (cmd == CMD_CLOSE)    return ESTADO_CERRANDO;
            if (cmd == CMD_TOGGLE)   return ESTADO_ABRIENDO;
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }
        tick_telemetria(); publicar_estado_si_cambia(); vTaskDelay(pdMS_TO_TICKS(20));
    }
}
static int loop_abriendo(void) {
    motor_abrir();
    uint64_t deadline_us = esp_timer_get_time() + (uint64_t)T_OPEN_MS * 1000ULL;
    publicar_estado_si_cambia();
    while (1) {
        leer_sensores();
        if (g_lsa && g_lsc) { motor_stop(); g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsa && !g_lsc) { motor_stop(); return ESTADO_ABIERTO; }
        if (esp_timer_get_time() > deadline_us) { motor_stop(); g_error_code = ERR_TIMEOUT_OPEN; return ESTADO_ERROR; }
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_STOP)   { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_CLOSE)  { motor_cerrar(); deadline_us = esp_timer_get_time() + (uint64_t)T_CLOSE_MS * 1000ULL; g_estado = ESTADO_CERRANDO; publicar_estado_si_cambia(); return ESTADO_CERRANDO; }
            if (cmd == CMD_TOGGLE) { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }
        tick_telemetria(); publicar_estado_si_cambia(); vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static int loop_cerrando(void) {
    motor_cerrar();
    uint64_t deadline_us = esp_timer_get_time() + (uint64_t)T_CLOSE_MS * 1000ULL;
    publicar_estado_si_cambia();
    while (1) {
        leer_sensores();
        if (g_lsa && g_lsc) { motor_stop(); g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
        if (g_lsc && !g_lsa) { motor_stop(); return ESTADO_CERRADO; }
        if (esp_timer_get_time() > deadline_us) { motor_stop(); g_error_code = ERR_TIMEOUT_CLOSE; return ESTADO_ERROR; }
        gate_cmd_t cmd;
        if (fetch_cmd(&cmd)) {
            if (cmd == CMD_STOP)   { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_OPEN)   { motor_abrir(); deadline_us = esp_timer_get_time() + (uint64_t)T_OPEN_MS * 1000ULL; g_estado = ESTADO_ABRIENDO; publicar_estado_si_cambia(); return ESTADO_ABRIENDO; }
            if (cmd == CMD_TOGGLE) { motor_stop(); return ESTADO_DETENIDO; }
            if (cmd == CMD_LAMP_ON)  lamp_on(true);
            if (cmd == CMD_LAMP_OFF) lamp_on(false);
        }
        tick_telemetria(); publicar_estado_si_cambia(); vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static int loop_inicial(void) {
    leer_sensores();
    if (g_lsa && g_lsc) { g_error_code = ERR_LS_INCONSISTENT; return ESTADO_ERROR; }
    if (g_lsa && !g_lsc) return ESTADO_ABIERTO;
    if (g_lsc && !g_lsa) return ESTADO_CERRADO;
    return ESTADO_DESCONOCIDO;
}

// ----------------------------- DISPATCHER FSM ---------------------------------
static void state_machine_task(void *arg) {
    g_last_pub_us = esp_timer_get_time();
    lamp_on(false); motor_stop();
    while (1) {
        switch (g_estado) {
            case ESTADO_INICIAL:     g_estado = loop_inicial();     publicar_estado_si_cambia(); break;
            case ESTADO_ABIERTO:     g_estado = loop_abierto();     break;
            case ESTADO_CERRADO:     g_estado = loop_cerrado();     break;
            case ESTADO_ABRIENDO:    g_estado = loop_abriendo();    break;
            case ESTADO_CERRANDO:    g_estado = loop_cerrando();    break;
            case ESTADO_DETENIDO:    g_estado = loop_detenido();    break;
            case ESTADO_DESCONOCIDO: g_estado = loop_desconocido(); break;
            case ESTADO_ERROR:       g_estado = loop_error();       break;
            default:                 g_estado = ESTADO_ERROR;       g_error_code = ERR_STATE_GUARDRAIL; break;
        }
    }
}

// ------------------------------ INICIALIZACIÓN --------------------------------
static void gpio_init_all(void) {
    gpio_config_t in = { .pin_bit_mask=(1ULL<<PIN_LSA)|(1ULL<<PIN_LSC), .mode=GPIO_MODE_INPUT, .pull_up_en=GPIO_PULLUP_DISABLE, .pull_down_en=GPIO_PULLDOWN_DISABLE, .intr_type=GPIO_INTR_DISABLE };
    ESP_ERROR_CHECK(gpio_config(&in));
    gpio_config_t out = { .pin_bit_mask=(1ULL<<PIN_MOTOR_A)|(1ULL<<PIN_MOTOR_C)|(1ULL<<PIN_LAMP), .mode=GPIO_MODE_OUTPUT, .pull_up_en=GPIO_PULLUP_DISABLE, .pull_down_en=GPIO_PULLDOWN_DISABLE, .intr_type=GPIO_INTR_DISABLE };
    ESP_ERROR_CHECK(gpio_config(&out));
    motor_stop(); lamp_on(false);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { ESP_ERROR_CHECK(nvs_flash_erase()); ESP_ERROR_CHECK(nvs_flash_init()); }

    gpio_init_all();
    wifi_init_sta();

    q_cmd = xQueueCreate(16, sizeof(gate_cmd_t));
    if (g_mqtt_uri[0]) mqtt_init();   // solo si hay broker configurado

    xTaskCreatePinnedToCore(state_machine_task, "state_machine_task", 4096, NULL, 10, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "Sistema iniciado.");
}
