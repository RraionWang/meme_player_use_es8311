#include "my_utils.h"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_vol.h"
#include "driver/i2c_types.h"
#include "driver/i2s_types.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "avi_player.h"
#include "esp_jpeg_dec.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "button_types.h"
#include "button_gpio.h"
#include "iot_button.h"
#include "driver/ledc.h"
#include "font8x16.h"
#include "battery_monitor.h"
#include "nvs.h"
#include "nvs_flash.h"

const char *video_list[] = {
    "/littlefs/oii.avi",
    "/littlefs/hajimi.avi",
    "/littlefs/happy.avi",
};

sys_cfg_t g_cfg;

int video_count = sizeof(video_list) / sizeof(video_list[0]);
int video_index = 0;

int bat_voltage = -1;
int bat_remian = -1;
int charg_status = -1;

#define EXAMPLE_RECV_BUF_SIZE (2400)
#define EXAMPLE_SAMPLE_RATE (16000)
#define EXAMPLE_MCLK_MULTIPLE (256) // If not using 24-bit data width, 256 should be enough
#define EXAMPLE_MCLK_FREQ_HZ (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)

#define EXAMPLE_PA_CTRL_IO 46

int example_voice_volime;

/* I2C port and GPIOs */
#define I2C_NUM (0)
#define I2C_SCL_IO 47
#define I2C_SDA_IO 21

/* I2S port and GPIOs */
#define I2S_NUM (0)
#define I2S_MCK_IO 40
#define I2S_BCK_IO 39
#define I2S_WS_IO 45
#define I2S_DO_IO 48
#define I2S_DI_IO 38

int lcd_lightness;

static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;

static const char err_reason[][30] = {"input param is invalid",
                                      "operation timeout"};

extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[] asm("_binary_canon_pcm_end");

static const char *TAG_FS = "esp_littlefs";
static const char *TAG_ES8311 = "es8311";
static const char *TAG_MUSIC = "music";
static const char *TAG_AVI = "avi";

avi_player_handle_t avi_player_handle;

bool isPause = false;

/**
 *
 */

void nvs_save_blob(const char *key, void *data, size_t len)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return;

    nvs_set_blob(handle, key, data, len);
    nvs_commit(handle);
    nvs_close(handle);
}

void nvs_load_blob(const char *key, void *data, size_t len)
{
    nvs_handle_t handle;
    size_t size = len;

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return;

    err = nvs_get_blob(handle, key, data, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        memset(data, 0, len);
    }

    nvs_close(handle);
}

/// nvs结束

void init_littlefs()
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG_FS, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG_FS, "Failed to find LittleFS partition");
        }
        else
        {
            ESP_LOGE(TAG_FS, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
}

// es8311

esp_codec_dev_handle_t codec_handle;

esp_err_t es8311_codec_init(void)
{
    /* Initialize I2C peripheral */
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_master_bus_config_t i2c_mst_cfg = {
        .i2c_port = I2C_NUM,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* Pull-up internally for no external pull-up case.
        Suggest to use external pull-up to ensure a strong enough pull-up. */
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_cfg, &i2c_bus_handle));

    /* Create control interface with I2C bus handle */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus_handle,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(ctrl_if);

    /* Create data interface with I2S bus handle */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM,
        .rx_handle = rx_handle,
        .tx_handle = tx_handle,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    assert(data_if);

    /* Create ES8311 interface handle */
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    assert(gpio_if);
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .master_mode = false,
        .use_mclk = I2S_MCK_IO >= 0,
        .pa_pin = EXAMPLE_PA_CTRL_IO,
        .pa_reverted = false,
        .hw_gain = {
            .pa_voltage = 3.3,
            .codec_dac_voltage = 3.3,
        },
        .mclk_div = EXAMPLE_MCLK_MULTIPLE,
    };
    const audio_codec_if_t *es8311_if = es8311_codec_new(&es8311_cfg);
    assert(es8311_if);

    /* Create the top codec handle with ES8311 interface handle and data interface */
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = es8311_if,
        .data_if = data_if,
    };
    codec_handle = esp_codec_dev_new(&dev_cfg);
    assert(codec_handle);

    /* Specify the sample configurations and open the device */
    esp_codec_dev_sample_info_t sample_cfg = {
        .bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT,
        .channel = 2,
        .channel_mask = 0x03,
        .sample_rate = EXAMPLE_SAMPLE_RATE,
    };
    if (esp_codec_dev_open(codec_handle, &sample_cfg) != ESP_CODEC_DEV_OK)
    {
        ESP_LOGE(TAG_ES8311, "Open codec device failed");
        return ESP_FAIL;
    }

    /* Set the initial volume and gain */
    if (esp_codec_dev_set_out_vol(codec_handle, g_cfg.volume) != ESP_CODEC_DEV_OK)
    {
        ESP_LOGE(TAG_ES8311, "set output volume failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t i2s_driver_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    return ESP_OK;
}

void i2s_music(void *args)
{
    esp_err_t ret = ESP_OK;
    size_t bytes_write = 0;
    uint8_t *data_ptr = (uint8_t *)music_pcm_start;

    /* (Optional) Disable TX channel and preload the data before enabling the TX channel,
     * so that the valid data can be transmitted immediately */
    ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_preload_data(tx_handle, data_ptr, music_pcm_end - data_ptr, &bytes_write));
    data_ptr += bytes_write; // Move forward the data pointer

    /* Enable the TX channel */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    while (1)
    {
        /* Write music to earphone */
        ret = i2s_channel_write(tx_handle, data_ptr, music_pcm_end - data_ptr, &bytes_write, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            /* Since we set timeout to 'portMAX_DELAY' in 'i2s_channel_write'
               so you won't reach here unless you set other timeout value,
               if timeout detected, it means write operation failed. */
            ESP_LOGE(TAG_MUSIC, "[music] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
            abort();
        }
        if (bytes_write > 0)
        {
            ESP_LOGI(TAG_MUSIC, "[music] i2s music played, %d bytes are written.", bytes_write);
        }
        else
        {
            ESP_LOGE(TAG_MUSIC, "[music] i2s music play failed.");
            abort();
        }
        data_ptr = (uint8_t *)music_pcm_start;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

#define LCD_HOST SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)

#define LCD_PIN_MOSI 6
#define LCD_PIN_CLK 7
#define LCD_PIN_CS 15
#define LCD_PIN_DC 16
#define LCD_PIN_RST 5
#define LCD_PIN_BK_LIGHT 4
#define LCD_H_RES 240
#define LCD_V_RES 240
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8
#define PARALLEL_LINES 16

static esp_lcd_panel_handle_t panel_handle = NULL;

void lcd_init()
{

    // 不使用这个使用ledc
    // if (LCD_PIN_BK_LIGHT >= 0) {
    //     gpio_set_direction(LCD_PIN_BK_LIGHT, GPIO_MODE_OUTPUT);
    //     gpio_set_level(LCD_PIN_BK_LIGHT, 0);
    // }

    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_CLK, .mosi_io_num = LCD_PIN_MOSI, .miso_io_num = -1, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = PARALLEL_LINES * LCD_H_RES * 2 + 8};
    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_disp_on_off(panel_handle, true);
    if (LCD_PIN_BK_LIGHT >= 0)
        gpio_set_level(LCD_PIN_BK_LIGHT, 1);
}

// 播放标志

static bool end_play = false;

/**
 * 显示文字工具函数
 */

static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) |
           ((g & 0xFC) << 3) |
           (b >> 3);
}

static inline void draw_pixel(uint16_t *buf, int buf_w, int x, int y, uint16_t color)
{
    buf[y * buf_w + x] = color;
}

// 修改后的 draw_char (按行存储)
void draw_char(uint16_t *buf, int buf_w, int x, int y, char c, uint16_t color)
{
    // bitmap 包含 16 个字节，每个字节代表一行 (8 像素)
    const uint8_t *bitmap = font8x16[(uint8_t)c];

    // 遍历 16 行 (高度)
    for (int row = 0; row < 16; row++)
    {
        uint8_t line_data = bitmap[row]; // 读取当前行的 8 个像素数据

        // 遍历 8 列 (宽度)
        for (int col = 0; col < 8; col++)
        {
            // 检查当前字节的第 col 位（从左到右或从右到左取决于字体生成工具）
            // 假设第 7 位是左边的第一个像素，第 0 位是右边的最后一个像素。
            if (line_data & (0x80 >> col))
            {
                draw_pixel(buf, buf_w, x + col, y + row, color);
            }
        }
    }
}

void draw_text(uint16_t *buf, int buf_w, int x, int y, const char *text, uint16_t color)
{
    while (*text)
    {
        draw_char(buf, buf_w, x, y, *text, color);
        x += 8; // 每个字符宽度 6 像素
        text++;
    }
}

void draw_osd_info(void)
{
    static uint16_t osd_buf[240 * 20];
    static bool osd_dirty = true;

    static int last_bright = -1;
    static int last_vol = -1;
    static int last_charg_status = false;

    // 检查是否真的有变化
    if (last_bright != lcd_lightness ||
        last_vol != example_voice_volime ||
        last_charg_status != charg_status)
    {
        osd_dirty = true;
    }

    if (osd_dirty)
    {
        // 更新记录
        last_bright = lcd_lightness;
        last_vol = example_voice_volime;
        last_charg_status = charg_status;

        // ⭐ 重新生成 OSD 画面
        for (int i = 0; i < 240 * 20; i++)
        {
            osd_buf[i] = RGB565(40, 40, 40);
        }

        char text[64];

        if (charg_status == -0)
        {
            snprintf(text, sizeof(text),
                     "Bri:%2d Vol:%2d BAT:%2d %s",
                     lcd_lightness,
                     example_voice_volime,
                     bat_remian,
                     "NCHG");
        }
        else if (charg_status == 1)
        {
            snprintf(text, sizeof(text),
                     "Bri:%2d Vol:%2d BAT:%2d %s",
                     lcd_lightness,
                     example_voice_volime,
                     bat_remian,
                     "CHGING");
        }
        else
        {
            snprintf(text, sizeof(text),
                     "Bri:%2d Vol:%2d BAT:%2d %s",
                     lcd_lightness,
                     example_voice_volime,
                     bat_remian,
                     "");
        }

        draw_text(osd_buf, 240, 1, 1, text, RGB565(255, 255, 255));

        osd_dirty = false;
    }

    // ⭐ 永远只画已有缓存（不会闪烁）
    esp_lcd_panel_draw_bitmap(panel_handle,
                              0, 0,
                              240, 20,
                              osd_buf);
}

/////////////////////////////

/**
 * 回调函数
 */

esp_err_t bsp_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_write(codec_handle, audio_buffer, len);
    *bytes_written = len;
    return ret;
}

void video_write(frame_data_t *data, void *arg)
{
    if (end_play)
    {
        avi_player_play_stop(avi_player_handle);
    }

    if (isPause)
    {
        return;
    }

    jpeg_dec_handle_t jpeg_dec_handle = NULL;
    jpeg_dec_io_t *jpeg_io = NULL;
    jpeg_dec_header_info_t header_info;
    esp_err_t err = ESP_OK;

    uint8_t *rgb_buffer = (uint8_t *)heap_caps_aligned_alloc(16, 240 * 240 * 2, MALLOC_CAP_DMA);
    if (rgb_buffer == NULL)
    {
        ESP_LOGE("VIDEO_CB", "无法为 RGB 对齐缓冲区分配内存");
        return;
    }

    jpeg_io = (jpeg_dec_io_t *)calloc(1, sizeof(jpeg_dec_io_t));
    if (jpeg_io == NULL)
    {
        ESP_LOGE("VIDEO_CB", "无法为 IO 结构体分配内存");
        free(rgb_buffer);
        return;
    }

    jpeg_dec_config_t jpeg_dec_cfg = {
        .output_type = JPEG_PIXEL_FORMAT_RGB565_BE,
        .rotate = JPEG_ROTATE_0D,
    };

    err = jpeg_dec_open(&jpeg_dec_cfg, &jpeg_dec_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("VIDEO_CB", "创建 JPEG 解码器失败 (0x%x)", err);
        goto exit;
    }

    jpeg_io->inbuf = data->data;
    jpeg_io->inbuf_len = data->data_bytes;
    jpeg_io->outbuf = rgb_buffer;

    err = jpeg_dec_parse_header(jpeg_dec_handle, jpeg_io, &header_info);
    if (err != ESP_OK)
    {
        ESP_LOGE("VIDEO_CB", "解析 JPEG 头部失败 (0x%x)", err);
        goto exit;
    }

    err = jpeg_dec_process(jpeg_dec_handle, jpeg_io);
    if (err == ESP_OK)
    {
        extern esp_lcd_panel_handle_t panel_handle;

        esp_lcd_panel_draw_bitmap(panel_handle, 0, 20, header_info.width, header_info.height, rgb_buffer);

        draw_osd_info();
    }
    else
    {
        ESP_LOGE("VIDEO_CB", "JPEG 解码失败 (0x%x)", err);
    }

exit:
    if (jpeg_dec_handle)
    {
        jpeg_dec_close(jpeg_dec_handle);
    }
    if (jpeg_io)
    {
        free(jpeg_io);
    }
    if (rgb_buffer)
    {
        free(rgb_buffer);
    }

    // ESP_LOGI(TAG_AVI, "Video write: %d", data->data_bytes);
}

void audio_write(frame_data_t *data, void *arg)
{
    size_t bytes_written = 0;

    if (isPause)
    {
        return;
    }
    bsp_i2s_write(data->data, data->data_bytes, &bytes_written, portMAX_DELAY);

    //   ESP_LOGI(TAG_AVI, "Audio write: %d", data->data_bytes);
}

esp_err_t bsp_codec_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,
        .channel = ch,
        .bits_per_sample = bits_cfg,
    };

    if (codec_handle)
    {
        ret = esp_codec_dev_close(codec_handle);
    }
    // if (record_dev_handle) {
    //     ret |= esp_codec_dev_close(record_dev_handle);
    //     ret |= esp_codec_dev_set_in_gain(record_dev_handle, CODEC_DEFAULT_ADC_VOLUME);
    // }

    if (codec_handle)
    {
        ret |= esp_codec_dev_open(codec_handle, &fs);
    }
    // if (record_dev_handle) {
    //     ret |= esp_codec_dev_open(record_dev_handle, &fs);
    // }
    return ret;
}

void audio_set_clock(uint32_t rate, uint32_t bits_cfg, uint32_t ch, void *arg)
{
    ESP_LOGI(TAG_AVI, "Audio set clock, rate %" PRIu32 ", bits %" PRIu32 ", ch %" PRIu32 "", rate, bits_cfg, ch);
    bsp_codec_set_fs(rate, bits_cfg, (i2s_slot_mode_t)ch);
}

void avi_play_end(void *arg)
{
    ESP_LOGI(TAG_AVI, "Play end");
    end_play = true;
}

void video_task(void *arg)
{
    while (1)
    {

        end_play = false;
        isPause = false;

        avi_player_config_t config = {
            .buffer_size = 60 * 1024,
            .audio_cb = audio_write,
            .video_cb = video_write,
            .audio_set_clock_cb = audio_set_clock,
            .avi_play_end_cb = avi_play_end,
            .stack_size = 4096,
            .stack_in_psram = false,
        };

        avi_player_init(config, &avi_player_handle);

        const char *file = video_list[video_index];
        ESP_LOGI("AVI", "开始播放: %s", file);

        avi_player_play_from_file(avi_player_handle, file);

        // 等待播放完毕 或 stop
        while (!end_play)
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }

        avi_player_deinit(avi_player_handle);

        // 自动切下一首
        video_index++;
        if (video_index >= video_count)
            video_index = 0;
    }
}

// 按键回调

static void button_single_click_vol_plus(void *arg, void *usr_data)
{
    example_voice_volime += 10;
    if (example_voice_volime >= 100)
    {
        example_voice_volime = 100;
    }
    esp_codec_dev_set_out_vol(codec_handle, example_voice_volime);

    ESP_LOGI("VOL", "音量增加，设置为: %d", example_voice_volime);

    g_cfg.volume = example_voice_volime;
    nvs_save_blob("syscfg", &g_cfg, sizeof(g_cfg));
}

static void button_single_click_vol_minus(void *arg, void *usr_data)
{
    example_voice_volime -= 10;
    if (example_voice_volime <= 0)
    {
        example_voice_volime = 0;
    }
    esp_codec_dev_set_out_vol(codec_handle, example_voice_volime);
    ESP_LOGI("VOL", "音量减少，设置为: %d", example_voice_volime);
    g_cfg.volume = example_voice_volime;
    nvs_save_blob("syscfg", &g_cfg, sizeof(g_cfg));
}

static void light_plus_cb(void *arg, void *usr_data)
{

    lcd_lightness += 10;
    if (lcd_lightness >= 100)
    {
        lcd_lightness = 100;
    }
    lcd_set_backlight_percent(lcd_lightness);

    ESP_LOGI("BLK", "亮度增加设置为: %d", lcd_lightness);
    g_cfg.brightness = lcd_lightness;
    nvs_save_blob("syscfg", &g_cfg, sizeof(g_cfg));
}

static void light_plus_minus(void *arg, void *usr_data)
{

    lcd_lightness -= 10;
    if (lcd_lightness <= 0)
    {
        lcd_lightness = 0;
    }
    lcd_set_backlight_percent(lcd_lightness);

    ESP_LOGI("BLK", "亮度减少设置为: %d", lcd_lightness);
    g_cfg.brightness = lcd_lightness;
    nvs_save_blob("syscfg", &g_cfg, sizeof(g_cfg));
}

// 将亮度和音量写到一起

void button_vol_plus_init()
{
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = 1,
        .active_level = 0,
    };
    button_handle_t gpio_btn = NULL;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &gpio_btn);

    if (NULL == gpio_btn)
    {
        ESP_LOGE("BUT", "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, NULL, button_single_click_vol_plus, NULL);

    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, NULL, light_plus_cb, NULL);
    nvs_save_blob("syscfg", &g_cfg, sizeof(g_cfg));
}

void button_vol_minus_init()
{
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = 18,
        .active_level = 0,
    };
    button_handle_t gpio_btn = NULL;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &gpio_btn);

    if (NULL == gpio_btn)
    {
        ESP_LOGE("BUT", "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, NULL, button_single_click_vol_minus, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, NULL, light_plus_minus, NULL);
    nvs_save_blob("syscfg", &g_cfg, sizeof(g_cfg));
}

static void button_single_click_func(void *arg, void *usr_data)
{

    isPause = false;
    ESP_LOGE("BUT", "播放");
}

static void button_single_up_click_func(void *arg, void *usr_data)
{

    isPause = true;
    ESP_LOGE("BUT", "暂停");
}

// 功能按钮
void button_func_init()
{
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = 11,
        .active_level = 0,
    };
    button_handle_t gpio_btn = NULL;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &gpio_btn);

    if (NULL == gpio_btn)
    {
        ESP_LOGE("BUT", "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_PRESS_DOWN, NULL, button_single_click_func, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_PRESS_UP, NULL, button_single_up_click_func, NULL);
}

static void button_single_click_playnext(void *arg, void *usr_data)
{

    end_play = true;
    avi_player_play_stop(avi_player_handle);
    // video_index++;
    // if (video_index >= video_count) {
    //     video_index = 0;     // 循环播放
    // }

    ESP_LOGI("BUT", "切换到下一个视频");

    // // 停止当前播放
    // end_play = true;       // 让 avi_play 的 while 立即退出
    // isPause = false;       // 取消暂停状态
}

void button_next_init()
{
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = 2,
        .active_level = 0,
    };
    button_handle_t gpio_btn = NULL;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &gpio_btn);

    if (NULL == gpio_btn)
    {
        ESP_LOGE("BUT", "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, NULL, button_single_click_playnext, NULL);
}

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (4) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (8192)                // 默认设置亮度为最大
#define LEDC_FREQUENCY (4000)           // Frequency in Hertz. Set frequency at 4 kHz

void lcd_set_backlight_percent(int percent)
{
    if (percent < 0)
        percent = 0;
    if (percent > 100)
        percent = 100;

    // 将百分比转换为 0~1023 的 duty
    int duty = (8192 * percent) / 100;

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void init_ledc()
{
    nvs_load_blob("syscfg", &g_cfg, sizeof(g_cfg));

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    lcd_set_backlight_percent(g_cfg.brightness);
}
