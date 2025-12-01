#include <stdio.h>
#include "my_utils.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static const char* TAG = "main" ;

void app_main(void)
{
init_littlefs() ; 


 if (i2s_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "i2s driver init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "i2s driver init success");
    }
    /* Initialize i2c peripheral and config es8311 codec by i2c */
    if (es8311_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es8311 codec init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "es8311 codec init success");
    }

xTaskCreate(i2s_music, "i2s_music", 4096, NULL, 5, NULL);


}