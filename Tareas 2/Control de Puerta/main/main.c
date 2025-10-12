/*
 * Puerta con FSM + MQTT (parafraseado)
 * Reglas de negocio (idénticas):
 * - Estados: ABRIENDO, CERRANDO, ABIERTO, CERRADO
 * - LED:
 *     ABIERTO  -> encendido constante
 *     CERRADO  -> apagado constante
 *     ABRIENDO -> parpadeo rápido “prendiendo” (comienza ON)
 *     CERRANDO -> parpadeo rápido “apagando” (comienza OFF)
 * - Comandos:
 *     abrir  -> error si ya está ABIERTO; si se está moviendo, ignora
 *     cerrar -> error si ya está CERRADO; si se está moviendo, ignora
 * - emergencia -> congela el sistema hasta reiniciar el ESP32
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "protocol_examples_common.h"

#include "mqtt_client.h"
#include "driver/gpio.h"

/* -------------------- Config -------------------- */
#define LOG_TAG          "PUERTA_FSM_MQTT"
#define LED_PIN          GPIO_NUM_2
#define TICKS_VIAJE      30                   // duración simulada del movimiento
#define BLINK_PERIOD_MS  100                  // periodo de parpadeo rápido

/* Broker/Tópicos (mismo broker y tópicos del original) */
#define MQTT_URI         "ws://broker.emqx.io:8083/mqtt"
#define MQTT_USER        "easy-learning"
#define MQTT_PASS        "demo-para-el-canal"
#define TOPIC_CMD        "easy-learning/puerta/cmd"
#define TOPIC_STATUS     "easy-learning/puerta/status"

/* -------------------- Estados -------------------- */
typedef enum {
    ST_OPENING = 0,
    ST_CLOSING = 1,
    ST_OPEN    = 2,
    ST_CLOSED  = 3,
} door_state_t;

/* -------------------- Contexto -------------------- */
typedef struct {
    volatile door_state_t current;
    volatile door_state_t target;
    volatile int          move_ticks;
    volatile bool         emergency;
    volatile bool         blink_phase;
    volatile bool         start_open_anim;
    volatile bool         start_close_anim;
    esp_mqtt_client_handle_t client;
} controller_t;

static controller_t g = {
    .current          = ST_CLOSED,
    .target           = ST_CLOSED,
    .move_ticks       = 0,
    .emergency        = false,
    .blink_phase      = false,
    .start_open_anim  = true,
    .start_close_anim = false,
    .client           = NULL
};

/* -------------------- Prototipos -------------------- */
static void app_led_init(void);
static void mqtt_begin(void);
static void mqtt_publish_state(const char *estado, const char *detalle);
static void mqtt_evt(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void task_led(void *arg);
static void task_fsm(void *arg);

/* -------------------- MQTT -------------------- */
static void mqtt_evt(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(LOG_TAG, "MQTT conectado");
        esp_mqtt_client_subscribe(g.client, TOPIC_CMD, 0);
        mqtt_publish_state("arranque", "listo");
        break;

    case MQTT_EVENT_DATA: {
        char topic[64] = {0};
        char data[64]  = {0};
        snprintf(topic, sizeof(topic), "%.*s", ev->topic_len, ev->topic);
        snprintf(data,  sizeof(data),  "%.*s", ev->data_len,  ev->data);
        ESP_LOGI(LOG_TAG, "CMD: %s -> %s", topic, data);

        if (g.emergency) {
            mqtt_publish_state("error", "emergencia_activa_reinicie");
            break;
        }

        if (strcmp(data, "emergencia") == 0) {
            g.emergency = true;
            mqtt_publish_state("emergencia", "sistema_congelado_reinicie");
            break;
        }

        if (g.current == ST_OPENING || g.current == ST_CLOSING) {
            mqtt_publish_state("ocupado", "espera_que_termine");
            break;
        }

        if (strcmp(data, "abrir") == 0) {
            if (g.current == ST_OPEN) {
                mqtt_publish_state("error", "ya_estaba_abierto");
            } else {
                g.target = ST_OPEN;
            }
        } else if (strcmp(data, "cerrar") == 0) {
            if (g.current == ST_CLOSED) {
                mqtt_publish_state("error", "ya_estaba_cerrado");
            } else {
                g.target = ST_CLOSED;
            }
        } else {
            mqtt_publish_state("error", "cmd_desconocido");
        }
    } break;

    default:
        break;
    }
}

static void mqtt_begin(void)
{
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS
    };
    g.client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(g.client, ESP_EVENT_ANY_ID, mqtt_evt, NULL);
    esp_mqtt_client_start(g.client);
}

static void mqtt_publish_state(const char *estado, const char *detalle)
{
    if (!g.client) return;

    char payload[128];
    if (detalle && detalle[0]) {
        snprintf(payload, sizeof(payload),
                 "{\"estado\":\"%s\",\"detalle\":\"%s\"}", estado, detalle);
    } else {
        snprintf(payload, sizeof(payload),
                 "{\"estado\":\"%s\"}", estado);
    }
    esp_mqtt_client_publish(g.client, TOPIC_STATUS, payload, 0, 0, 0);
}

/* -------------------- LED -------------------- */
static void app_led_init(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
}

/* Indicación visual del estado */
static void task_led(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        if (g.emergency) {                // congelado: deja el LED como esté
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        switch (g.current) {

        case ST_OPEN:
            gpio_set_level(LED_PIN, 1);   // ON fijo
            vTaskDelay(pdMS_TO_TICKS(200));
            break;

        case ST_CLOSED:
            gpio_set_level(LED_PIN, 0);   // OFF fijo
            vTaskDelay(pdMS_TO_TICKS(200));
            break;

        case ST_OPENING:
            // arranca ON y luego alterna cada 100 ms
            g.blink_phase = !g.blink_phase;
            if (g.start_open_anim) {
                gpio_set_level(LED_PIN, 1);
                g.start_open_anim = false;
            } else {
                gpio_set_level(LED_PIN, g.blink_phase ? 1 : 0);
            }
            vTaskDelayUntil(&last, pdMS_TO_TICKS(BLINK_PERIOD_MS));
            break;

        case ST_CLOSING:
            // arranca OFF y luego alterna cada 100 ms (invertido)
            g.blink_phase = !g.blink_phase;
            if (!g.start_close_anim) {
                gpio_set_level(LED_PIN, 0);
                g.start_close_anim = true;
            } else {
                gpio_set_level(LED_PIN, g.blink_phase ? 0 : 1);
            }
            vTaskDelayUntil(&last, pdMS_TO_TICKS(BLINK_PERIOD_MS));
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        }
    }
}

/* -------------------- FSM -------------------- */
static void task_fsm(void *arg)
{
    for (;;) {
        if (g.emergency) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Arranque de movimiento si hay cambio de objetivo estando quieto
        if (g.current == ST_OPEN && g.target == ST_CLOSED) {
            g.current = ST_CLOSING;
            g.move_ticks = TICKS_VIAJE;
            g.start_close_anim = false;     // empieza “apagando”
            mqtt_publish_state("cerrando", "");
        } else if (g.current == ST_CLOSED && g.target == ST_OPEN) {
            g.current = ST_OPENING;
            g.move_ticks = TICKS_VIAJE;
            g.start_open_anim = true;       // empieza “prendiendo”
            mqtt_publish_state("abriendo", "");
        }

        // En viaje: cuenta y termina
        if (g.current == ST_OPENING || g.current == ST_CLOSING) {
            if (g.move_ticks > 0) g.move_ticks--;
            if (g.move_ticks <= 0) {
                if (g.current == ST_OPENING) {
                    g.current = ST_OPEN;
                    g.target  = ST_OPEN;
                    mqtt_publish_state("abierto", "");
                } else {
                    g.current = ST_CLOSED;
                    g.target  = ST_CLOSED;
                    mqtt_publish_state("cerrado", "");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* -------------------- main -------------------- */
void app_main(void)
{
    ESP_LOGI(LOG_TAG, "Inicializando sistema");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());     // Wi-Fi

    app_led_init();
    mqtt_begin();

    // Estado inicial
    g.current = ST_CLOSED;
    g.target  = ST_CLOSED;
    mqtt_publish_state("cerrado", "inicio");

    // Tareas
    xTaskCreate(task_led, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(task_fsm, "fsm_task", 2048, NULL, 6, NULL);
}
