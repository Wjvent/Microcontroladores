#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"

#define bombillodelesp32 2 // pin del led

uint8_t led_level = 0;
static const char *tag = "Main";
TimerHandle_t xTimers;
int interval = 50; // tiempo en ms
int timerId = 1;

esp_err_t init_led(void);
esp_err_t blink_led(void);
esp_err_t set_timer(void);

// función del timer
void vTimerCallback(TimerHandle_t pxTimer)
{
    ESP_LOGI(tag, "Event was called from timer");
    blink_led(); // parpadea el led
}

void app_main(void)
{
    init_led();  // inicializa el led
    set_timer(); // configura el timer
}

// configuración del pin
esp_err_t init_led(void)
{
    gpio_reset_pin(bombillodelesp32);
    gpio_set_direction(bombillodelesp32, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

// cambia el estado del led
esp_err_t blink_led(void)
{
    led_level = !led_level;
    gpio_set_level(bombillodelesp32, led_level);
    return ESP_OK;
}

// configuración del timer
esp_err_t set_timer(void)
{
    ESP_LOGI(tag, "Timer init configuration");
    xTimers = xTimerCreate("Timer",                   
                           (pdMS_TO_TICKS(interval)), 
                           pdTRUE,                    
                           (void *)timerId,           
                           vTimerCallback);           

    if (xTimers == NULL)
    {
        ESP_LOGE(tag, "The timer was not created.");
    }
    else
    {
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            ESP_LOGE(tag, "The timer could not be set into the Active state");
        }
    }

    return ESP_OK;
}