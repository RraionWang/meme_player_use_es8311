#pragma once
#include "esp_err.h"
#include "stdbool.h"

typedef struct
{
   int brightness;
   int volume;
   bool mute;
   int full_voltage;
} sys_cfg_t;

extern sys_cfg_t g_cfg;

extern int example_voice_volime;
extern int lcd_lightness;

void init_littlefs();

void nvs_load_blob(const char *key, void *data, size_t len);
void nvs_save_blob(const char *key, void *data, size_t len);

extern int bat_voltage;
extern int bat_remian;
extern int charg_status;

esp_err_t es8311_codec_init(void);
esp_err_t i2s_driver_init(void);
void i2s_music(void *args);
void lcd_init();



void draw_osd_info(void); 
 
void video_task(void *arg);

void button_vol_plus_init();

void button_vol_minus_init();

void button_func_init();

void button_next_init();

// 初始化pwm控制器
void init_ledc();
void lcd_set_backlight_percent(int percent);
