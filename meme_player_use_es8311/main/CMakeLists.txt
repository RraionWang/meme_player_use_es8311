idf_component_register(SRCS "my_utils.c" "main.c" 
  EMBED_FILES "canon.pcm"
       PRIV_REQUIRES esp_driver_i2s esp_driver_i2c esp_driver_gpio
                    INCLUDE_DIRS ".")