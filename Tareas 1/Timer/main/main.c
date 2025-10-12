#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "WORDS_ROTATOR";

/* --- Datos a rotar --- */
static const char *kWords[] = {
    "Energias Renovables",
    "Telecomunicaciones",
    "Mecatronica"
};
enum { kWordsCount = sizeof(kWords) / sizeof(kWords[0]) };

/* --- NVS --- */
static const char *kNvsNamespace = "storage";
static const char *kIndexKey     = "current_index";
static nvs_handle_t s_nvs = 0;

/* --- Declaraciones --- */
static void words_show(int32_t idx);
static int32_t words_next(int32_t idx);
static esp_err_t nvs_boot(void);
static int32_t nvs_read_index_default0(void);
static void nvs_write_index(int32_t idx);

/* ---------- Implementación ---------- */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_boot());

    int32_t pos = nvs_read_index_default0();
    if (pos < 0 || pos >= (int32_t)kWordsCount) {
        ESP_LOGW(TAG, "Indice fuera de rango (%ld). Reiniciando a 0.", (long)pos);
        pos = 0;
    }

    for (;;) {
        words_show(pos);               // Muestra palabra actual
        pos = words_next(pos);         // Calcula siguiente posición (cíclica)
        nvs_write_index(pos);          // Persiste el índice para el próximo arranque
        vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms
    }
}

/* Inicializa NVS y abre el espacio de trabajo */
static esp_err_t nvs_boot(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = nvs_open(kNvsNamespace, NVS_READWRITE, &s_nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo abrir NVS (%s)", esp_err_to_name(err));
    }
    return err;
}

/* Lee el indice guardado; si no existe o hay error, devuelve 0 */
static int32_t nvs_read_index_default0(void)
{
    int32_t idx = 0;
    esp_err_t err = nvs_get_i32(s_nvs, kIndexKey, &idx);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Indice no presente en NVS; iniciando en 0.");
        idx = 0;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo indice: %s. Usando 0.", esp_err_to_name(err));
        idx = 0;
    }
    return idx;
}

/* Escribe el indice y confirma el commit */
static void nvs_write_index(int32_t idx)
{
    esp_err_t err = nvs_set_i32(s_nvs, kIndexKey, idx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error nvs_set_i32: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_commit(s_nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error nvs_commit: %s", esp_err_to_name(err));
    }
}

/* Muestra la palabra de la posición dada */
static void words_show(int32_t idx)
{
    // Seguridad: clamp por si llega un valor inesperado
    if (idx < 0 || idx >= (int32_t)kWordsCount) idx = 0;
    ESP_LOGI(TAG, "%s", kWords[idx]);
}

/* Calcula el siguiente indice cíclico */
static int32_t words_next(int32_t idx)
{
    idx++;
    if (idx >= (int32_t)kWordsCount) idx = 0;
    return idx;
}
