#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* Pines de salida */
#define LED_A   GPIO_NUM_2
#define LED_B   GPIO_NUM_25
#define LED_C   GPIO_NUM_26

/* Configuración de pilas y periodos */
#define TASK_STACK      (1024 * 2)
#define PERIOD_LED_A_MS 1000
#define PERIOD_LED_B_MS 2000
#define PERIOD_LED_C_MS 4000

static const char *TAG = "BLINK_MULTI";

/* Descriptor para parametrizar una tarea de parpadeo */
typedef struct {
    gpio_num_t pin;
    TickType_t period_ticks;
    esp_log_level_t level;
    const char *name;
} blink_cfg_t;

/* Prototipos */
static esp_err_t gpio_init_outputs(gpio_num_t const *pins, size_t n);
static void blink_task(void *arg);

void app_main(void)
{
    /* 1) Configurar GPIOs como salida */
    const gpio_num_t outs[] = { LED_A, LED_B, LED_C };
    ESP_ERROR_CHECK(gpio_init_outputs(outs, sizeof(outs)/sizeof(outs[0])));

    /* 2) Crear tres tareas con distinta configuración */
    static blink_cfg_t c1 = { LED_A, pdMS_TO_TICKS(PERIOD_LED_A_MS), ESP_LOG_INFO,  "LED1" };
    static blink_cfg_t c2 = { LED_B, pdMS_TO_TICKS(PERIOD_LED_B_MS), ESP_LOG_WARN,  "LED2" };
    static blink_cfg_t c3 = { LED_C, pdMS_TO_TICKS(PERIOD_LED_C_MS), ESP_LOG_ERROR, "LED3" };

    xTaskCreate(blink_task, "blink_led1", TASK_STACK, &c1, 1, NULL);
    xTaskCreate(blink_task, "blink_led2", TASK_STACK, &c2, 1, NULL);
    xTaskCreate(blink_task, "blink_led3", TASK_STACK, &c3, 1, NULL);

    /* 3) Tarea implícita: imprimir algo periódico en el hilo principal */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        printf("klk pichardo, renovable en la casa\n");
    }
}

/* ---------- Implementaciones ---------- */

static esp_err_t gpio_init_outputs(gpio_num_t const *pins, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
    return ESP_OK;
}

/* Tarea genérica: parpadea un pin con un periodo y nivel de log dados */
static void blink_task(void *arg)
{
    const blink_cfg_t *cfg = (const blink_cfg_t *)arg;
    int level = 0;

    for (;;) {
        level = !level;
        gpio_set_level(cfg->pin, level);

        /* Log en el nivel solicitado */
        switch (cfg->level) {
            case ESP_LOG_INFO:  ESP_LOGI(TAG, "%s toggle -> %d", cfg->name, level); break;
            case ESP_LOG_WARN:  ESP_LOGW(TAG, "%s toggle -> %d", cfg->name, level); break;
            case ESP_LOG_ERROR: ESP_LOGE(TAG, "%s toggle -> %d", cfg->name, level); break;
            default:            ESP_LOGD(TAG, "%s toggle -> %d", cfg->name, level); break;
        }

        vTaskDelay(cfg->period_ticks);
    }
}
