#include <stdio.h>
#include "my_utils.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "battery_monitor.h"
#include "nvs.h"
#include "nvs_flash.h"







static const char* TAG = "main" ;

void app_main(void)
{


    // 初始化nvs
    esp_err_t err = nvs_flash_init();

    
if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
}



ESP_ERROR_CHECK(err);


// 载入信息
nvs_load_blob("syscfg", &g_cfg, sizeof(g_cfg)); 

ESP_LOGI("NVS","载入参数成功 vol =%d bri =%d vtg =%d",g_cfg.volume,g_cfg.brightness ,g_cfg.full_voltage) ; 



   example_voice_volime  = g_cfg.volume ; 
  lcd_lightness = g_cfg.brightness  ; 





init_littlefs() ; 


// 初始化调光系统
init_ledc() ;

// 初始化电池检测
battery_monitor_init() ;



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
lcd_init() ; 
  


 button_vol_plus_init() ;
 button_vol_minus_init() ; 
  button_func_init() ; 
button_next_init() ; 


// xTaskCreate(i2s_music, "i2s_music", 4096, NULL, 5, NULL);
xTaskCreate(video_task, "video_task", 8192, NULL, 5, NULL);



nvs_save_blob("syscfg", &g_cfg, sizeof(g_cfg));

}