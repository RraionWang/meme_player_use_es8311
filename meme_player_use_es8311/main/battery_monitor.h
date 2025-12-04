#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- 新增电池状态结构体 ---
typedef struct {
    int voltage_mv;       ///< 电池电压 (mV)
    int percentage;       ///< 电池电量百分比 (0-100, -1表示未校准/错误)
    bool is_charging;     ///< 是否正在充电 (CHARGING_GPIO == 0 且 DONE_GPIO == 1)
    bool is_full;         ///< 充电是否充满 (CHARGING_GPIO == 1 且 DONE_GPIO == 0)
    bool is_discharging;  ///< 是否正在放电 (未充电/未充满)
} battery_status_t;
// ----------------------------

/**
 * @brief 初始化电池ADC、充电检测GPIO和监控任务。
 *
 * @return ESP_OK 成功, 否则失败。
 */
esp_err_t battery_monitor_init(void);

/**
 * @brief 获取最后一次读取的完整电池状态。
 *
 * @param status 指向存储完整状态的结构体的指针。
 * @return true 成功获取, false 失败。
 */
bool get_battery_status(battery_status_t *status);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_MONITOR_H