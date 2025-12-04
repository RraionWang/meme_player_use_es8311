#include "battery_monitor.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <driver/gpio.h>
#include <freertos/semphr.h> // 新增：用于保护共享资源
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "my_utils.h"

// --- 配置常量 (请根据您的 `config.h` 或其他定义文件进行调整) ---
#define BATTERY_ADC_CHANNEL    ADC_CHANNEL_2
#define BATTERY_ADC_ATTEN      ADC_ATTEN_DB_12
#define BATTERY_READ_INTERVAL_MS (10 * 1000) // 1 分钟 (60000 ms)

// 电池电压范围 (假设 3.0V - 4.2V)
#define BATTERY_MIN_VOLTAGE_MV 3000


int battery_max_voltage_mv = 4200 ;



// 充电状态 GPIO (取自您的 gezipai_board[1].cc 中的逻辑)
// 请确保在您的配置头文件中定义了这些宏
#define CHARGING_GPIO 41 // 假设值，请替换为您的实际定义
#define DONE_GPIO     42 // 假设值，请替换为您的实际定义

// -----------------------------------------------------------------

#define TAG "BATT_MON"

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_calibrated = false;

// 存储最后读取的状态
static volatile battery_status_t current_battery_status = {0, -1, false, false, true};

// 互斥锁，用于保护 current_battery_status
static SemaphoreHandle_t batt_status_mutex = NULL;


/**
 * @brief 计算电池电量百分比
 * ... (与之前相同，省略)
 */
static int calculate_battery_percentage(int voltage)
{
    battery_max_voltage_mv = g_cfg.full_voltage ; 

    if (!adc_calibrated || voltage <= 0) {
        return -1;
    }

    if (voltage < BATTERY_MIN_VOLTAGE_MV) {
        return 0;
    } else if (voltage > battery_max_voltage_mv) {
        return 100;
    } else {
        return (voltage - BATTERY_MIN_VOLTAGE_MV) * 100 / (battery_max_voltage_mv - BATTERY_MIN_VOLTAGE_MV);
    }
}

/**
 * @brief 读取电池电压 (mV)
 * ... (与之前相同，省略)
 */
static int read_battery_voltage(void)
{
    int adc_raw = 0;
    long long sum = 0;
    int voltage = 0;
    const int num_samples = 10;

    for (int i = 0; i < num_samples; i++)
    {
        if (adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &adc_raw) == ESP_OK) {
            sum += adc_raw;
        } else {
            ESP_LOGE(TAG, "ADC 读取失败");
            return 0;
        }
    }
    adc_raw = sum / num_samples;

    if (adc_calibrated)
    {
        if (adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage) != ESP_OK) {
            ESP_LOGE(TAG, "ADC 校准转换失败");
            return 0;
        }
    } else {
        ESP_LOGW(TAG, "ADC 未校准，无法获取准确电压");
        return 0;
    }

    // 分压电阻 1:2，转换系数为 3
    return voltage * 3;
}

/**
 * @brief 读取充电状态
 * @param status 指向要更新的结构体
 */
static void update_charge_status(battery_status_t *status)
{
    uint8_t charge_level = gpio_get_level(CHARGING_GPIO);
    uint8_t done_level = gpio_get_level(DONE_GPIO);
    
    // 根据您的 C++ 原始逻辑：
    // CHARGING_GPIO 和 DONE_GPIO 可能是低电平有效 (Low-Active)
    /*
        if (chargeLevel == 0 && doneLevel == 0) {
            charging = false;
            discharging = true;
        } else if (chargeLevel == 0 && doneLevel == 1) {
            charging = true;
            discharging = false;
        } else if (chargeLevel == 1 && doneLevel == 0) {
            charging = false;
            discharging = true;
        }
    */
   
    // 逻辑简化及推导：
    if (charge_level == 0 && done_level == 1) {
        // CHARGING_LOW, DONE_HIGH -> 正在充电
        status->is_charging = true;
        status->is_full = false;
        status->is_discharging = false;
        ESP_LOGI(TAG, "充电状态: 正在充电");
        charg_status = true ; 
        
    } else if (charge_level == 1 && done_level == 0) {
        // CHARGING_HIGH, DONE_LOW -> 充电完成 (充满)
        status->is_charging = false;
        status->is_full = true;
        status->is_discharging = false;
        ESP_LOGI(TAG, "充电状态: 充电完成 (充满)");
        charg_status = false ; 
    } else {
        // 其他情况 (0,0 或 1,1) -> 默认视为放电或未连接
        status->is_charging = false;
        status->is_full = false;
        status->is_discharging = true;
        ESP_LOGI(TAG, "充电状态: 正在放电/未连接充电器");
        
    }
}


/**
 * @brief 电池监控 FreeRTOS 任务
 */
static void battery_monitor_task(void *pvParameters)
{
    battery_status_t new_status;

    while (1)
    {
        // 1. 读取电压和电量
        int new_voltage = read_battery_voltage();
        int new_percentage = calculate_battery_percentage(new_voltage);

        bat_remian = new_percentage ; 
        
        // 2. 读取充电状态
        update_charge_status(&new_status);

        // 3. 更新状态结构体
        new_status.voltage_mv = new_voltage;
        new_status.percentage = new_percentage;

        // 4. 使用互斥锁保护共享变量的更新
        if (xSemaphoreTake(batt_status_mutex, portMAX_DELAY) == pdTRUE)
        {
            current_battery_status = new_status;
            xSemaphoreGive(batt_status_mutex);
        }

        if (adc_calibrated) {
            ESP_LOGI(TAG, "电池状态更新 - 电压: %d mV, 电量: %d%%, 充电: %d, 充满: %d",
                     new_status.voltage_mv, new_status.percentage, 
                     new_status.is_charging, new_status.is_full);

                    



                                        
                    bat_remian  = new_status.percentage;

                    if( new_status.is_charging){
                        charg_status = 1 ; 
                    }else{
                        charg_status = 0 ;
                    }

                    if(new_status.is_full){
                        g_cfg.full_voltage = new_status.voltage_mv; 
                        nvs_save_blob("syscfg", &g_cfg, sizeof(g_cfg));
                        
                    }

                





        } else {
            ESP_LOGW(TAG, "电池ADC未校准，无法准确读取电压/电量");
        }

        vTaskDelay(pdMS_TO_TICKS(BATTERY_READ_INTERVAL_MS)); // 等待配置的间隔时间
    }
}

// -----------------------------------------------------------
// --- 公共 API 实现 ---
// -----------------------------------------------------------

/**
 * @brief 初始化充电状态检测 GPIO
 */
static esp_err_t init_charge_gpios()
{
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << CHARGING_GPIO) | (1ULL << DONE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // 假设外部没有或需要内部上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_config);
}


esp_err_t battery_monitor_init(void)
{
    esp_err_t ret = ESP_OK;

    // 1. 创建互斥锁
    batt_status_mutex = xSemaphoreCreateMutex();
    if (batt_status_mutex == NULL) {
        ESP_LOGE(TAG, "创建互斥锁失败");
        return ESP_FAIL;
    }

    // 2. 初始化充电 GPIO
    ret = init_charge_gpios();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "充电状态 GPIO 初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

   

    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1, };
    ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "ADC unit 初始化失败: %s", esp_err_to_name(ret)); return ret; }

    adc_cali_handle_t handle = NULL;
    ret = ESP_FAIL;
    adc_calibrated = false;

    
    // ... (ADC Channel 配置)
    adc_oneshot_chan_cfg_t config = { .atten = BATTERY_ADC_ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT, };
    ret = adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "ADC channel 配置失败: %s", esp_err_to_name(ret)); return ret; }
    
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!adc_calibrated)
    {
        ESP_LOGI(TAG, "尝试使用曲线拟合校准方案");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = BATTERY_ADC_CHANNEL,
            .atten = BATTERY_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            adc_calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!adc_calibrated)
    {
        ESP_LOGI(TAG, "尝试使用线性拟合校准方案");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = BATTERY_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            adc_calibrated = true;
        }
    }
#endif

    adc1_cali_handle = handle;

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC校准成功");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !adc_calibrated) {
        ESP_LOGW(TAG, "eFuse未烧录或校准方案不支持，跳过软件校准");
    } else {
        ESP_LOGE(TAG, "ADC 校准失败: %s", esp_err_to_name(ret));
    }




    // 4. 创建 FreeRTOS 任务进行周期性读取
    if (xTaskCreate(battery_monitor_task, "batt_mon_task", 4096, NULL, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "创建电池监控任务失败");
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool get_battery_status(battery_status_t *status)
{
    if (status == NULL) {
        return false;
    }
    
    // 使用互斥锁安全地读取共享变量
    if (xSemaphoreTake(batt_status_mutex, portMAX_DELAY) == pdTRUE)
    {
        *status = current_battery_status;
        xSemaphoreGive(batt_status_mutex);
    }
    
    return adc_calibrated; // 只有在校准成功的情况下，电量和电压数据才可信
}