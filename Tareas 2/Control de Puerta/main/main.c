// --- Sistema de control de puerta (FSM + MQTT) ---
// Definición de modos:
//   • ABRIENDO, CERRANDO, ABIERTO, CERRADO
//
// Indicador LED:
//   → ABIERTO: luz encendida permanente
//   → CERRADO: luz apagada permanente
//   → ABRIENDO: parpadeo rápido comenzando encendido
//   → CERRANDO: parpadeo rápido comenzando apagado
//
// Órdenes válidas:
//   - "abrir": se ignora si ya está abierto o en proceso
//   - "cerrar": se ignora si ya está cerrado o en proceso
//   - "emergencia": detiene toda acción hasta reinicio del dispositivo

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

// ===================================================
//                General Configuration
// ===================================================
#define LOG_TAG          "DOOR_CTRL_MQTT"
#define LED_PIN          GPIO_NUM_2
#define TICKS_TRAVEL     30          // simulated motion duration
#define BLINK_RATE_MS    100         // LED blink interval for transitions

// MQTT Broker / Topics (same parameters, different format)
#define MQTT_URI         "ws://broker.emqx.io:8083/mqtt"
#define MQTT_USER        "easy-learning"
#define MQTT_PASS        "demo-para-el-canal"
#define TOPIC_CMD        "easy-learning/puerta/cmd"
#define TOPIC_STATUS     "easy-learning/puerta/status"

// ===================================================
//                State Definitions
// ===================================================
typedef enum {
    ST_OPENING = 0,
    ST_CLOSING = 1,
    ST_OPEN    = 2,
    ST_CLOSED  = 3,
} door_state_t;

// ===================================================
//                Global Context
// ===================================================
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

// ===================================================
//                Function Prototypes
// ===================================================
static void init_led(void);
static void start_mqtt(void);
static void mqtt_send_status(const char *state, const char *info);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void led_task(void *arg);
static void fsm_task(void *arg);

// ===================================================
//                MQTT Section
// ===================================================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(LOG_TAG, "Connected to MQTT broker");
        esp_mqtt_client_subscribe(g.client, TOPIC_CMD, 0);
        mqtt_send_status("boot", "system_ready");
        break;

    case MQTT_EVENT_DATA: {
        char topic[64] = {0};
        char data[64]  = {0};
        snprintf(topic, sizeof(topic), "%.*s", ev->topic_len, ev->topic);
        snprintf(data,  sizeof(data),  "%.*s", ev->data_len,  ev->data);
        ESP_LOGI(LOG_TAG, "Received -> %s : %s", topic, data);

        if (g.emergency) {
            mqtt_send_status("error", "emergency_active_restart_required");
            break;
        }

        if (strcmp(data, "emergencia") == 0) {
            g.emergency = true;
            mqtt_send_status("emergency", "system_frozen_restart_needed");
            break;
        }

        if (g.current == ST_OPENING || g.current == ST_CLOSING) {
            mqtt_send_status("busy", "door_in_motion");
            break;
        }

        if (strcmp(data, "abrir") == 0) {
            if (g.current == ST_OPEN) {
                mqtt_send_status("error", "already_open");
            } else {
                g.target = ST_OPEN;
            }
        } else if (strcmp(data, "cerrar") == 0) {
            if (g.current == ST_CLOSED) {
                mqtt_send_status("error", "already_closed");
            } else {
                g.target = ST_CLOSED;
            }
        } else {
            mqtt_send_status("error", "unknown_command");
        }
    } break;

    default:
        break;
    }
}

static void start_mqtt(void)
{
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS
    };
    g.client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(g.client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(g.client);
}

static void mqtt_send_status(const char *state, const char *info)
{
    if (!g.client) return;

    char payload[128];
    if (info && info[0]) {
        snprintf(payload, sizeof(payload),
                 "{\"state\":\"%s\",\"info\":\"%s\"}", state, info);
    } else {
        snprintf(payload, sizeof(payload),
                 "{\"state\":\"%s\"}", state);
    }
    esp_mqtt_client_publish(g.client, TOPIC_STATUS, payload, 0, 0, 0);
}

// ===================================================
//                LED Section
// ===================================================
static void init_led(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
}

// Handles LED feedback depending on door state
static void led_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        if (g.emergency) {  // during emergency keep LED frozen
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        switch (g.current) {

        case ST_OPEN:
            gpio_set_level(LED_PIN, 1);  // solid ON
            vTaskDelay(pdMS_TO_TICKS(200));
            break;

        case ST_CLOSED:
            gpio_set_level(LED_PIN, 0);  // solid OFF
            vTaskDelay(pdMS_TO_TICKS(200));
            break;

        case ST_OPENING:
            // starts ON, toggles quickly
            g.blink_phase = !g.blink_phase;
            if (g.start_open_anim) {
                gpio_set_level(LED_PIN, 1);
                g.start_open_anim = false;
            } else {
                gpio_set_level(LED_PIN, g.blink_phase ? 1 : 0);
            }
            vTaskDelayUntil(&last, pdMS_TO_TICKS(BLINK_RATE_MS));
            break;

        case ST_CLOSING:
            // starts OFF, toggles quickly (inverse pattern)
            g.blink_phase = !g.blink_phase;
            if (!g.start_close_anim) {
                gpio_set_level(LED_PIN, 0);
                g.start_close_anim = true;
            } else {
                gpio_set_level(LED_PIN, g.blink_phase ? 0 : 1);
            }
            vTaskDelayUntil(&last, pdMS_TO_TICKS(BLINK_RATE_MS));
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        }
    }
}

// ===================================================
//                FSM Core Task
// ===================================================
static void fsm_task(void *arg)
{
    for (;;) {
        if (g.emergency) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Initiate movement if target changes while idle
        if (g.current == ST_OPEN && g.target == ST_CLOSED) {
            g.current = ST_CLOSING;
            g.move_ticks = TICKS_TRAVEL;
            g.start_close_anim = false;
            mqtt_send_status("closing", "");
        } else if (g.current == ST_CLOSED && g.target == ST_OPEN) {
            g.current = ST_OPENING;
            g.move_ticks = TICKS_TRAVEL;
            g.start_open_anim = true;
            mqtt_send_status("opening", "");
        }

        // During travel, countdown and switch final state
        if (g.current == ST_OPENING || g.current == ST_CLOSING) {
            if (g.move_ticks > 0) g.move_ticks--;
            if (g.move_ticks <= 0) {
                if (g.current == ST_OPENING) {
                    g.current = ST_OPEN;
                    g.target  = ST_OPEN;
                    mqtt_send_status("open", "");
                } else {
                    g.current = ST_CLOSED;
                    g.target  = ST_CLOSED;
                    mqtt_send_status("closed", "");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ===================================================
//                Main Application Entry
// ===================================================
void app_main(void)
{
    ESP_LOGI(LOG_TAG, "System boot sequence started");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());  // connect Wi-Fi

    init_led();
    start_mqtt();

    // Initial door state
    g.current = ST_CLOSED;
    g.target  = ST_CLOSED;
    mqtt_send_status("closed", "startup");

    // Task creation
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(fsm_task, "fsm_task", 2048, NULL, 6, NULL);
}
