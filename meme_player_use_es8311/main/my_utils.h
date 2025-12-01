#pragma once 
#include "esp_err.h"
void init_littlefs() ; 

 esp_err_t es8311_codec_init(void);
  esp_err_t i2s_driver_init(void);
   void i2s_music(void *args) ; 