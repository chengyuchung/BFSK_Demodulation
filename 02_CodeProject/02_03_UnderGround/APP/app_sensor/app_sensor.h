/**
 * @file    app_sensor.h
 * @brief   传感器管理 - APP 层
 * @note    向上层（BSW Scheduler / FSM）提供统一温度接口
 *          负责采样周期控制、异常过滤
 */

#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsw_ds18b20.h"

/* ========== 配置参数 ========== */

/* 温度采样周期，单位：ms（默认 5 秒） */
#ifndef APP_SENSOR_SAMPLE_PERIOD_MS
#define APP_SENSOR_SAMPLE_PERIOD_MS   5000
#endif

/* 温度异常阈值（超过此值认为是异常，单位同 raw_value = ×0.01°C） */
#define APP_SENSOR_TEMP_MIN            (-4000)   /* -40°C */
#define APP_SENSOR_TEMP_MAX            (12500)   /* 125°C */

/* ========== 传感器数据（供外部只读访问） ========== */
typedef struct {
    int16_t   temperature;       /* 温度值（×0.01°C），如 2550=25.50°C */
    uint32_t  last_update;       /* 上次更新时刻（HAL_GetTick） */
    uint8_t   valid;             /* 数据有效标志：1=有效 0=无效 */
    uint8_t   online;             /* 设备在线标志：1=在线 0=离线 */
} app_sensor_data_t;

/* ========== 函数声明 ========== */

/**
 * @brief   传感器 APP 层初始化
 * @note    在系统启动时调用一次
 */
void app_sensor_init(void);

/**
 * @brief   传感器周期性任务（由 Scheduler 调用）
 * @note    内部判断采样周期，控制 BSW 层触发采样
 * @param   now   当前时刻（HAL_GetTick）
 */
void app_sensor_task(uint32_t now);

/**
 * @brief   获取当前温度数据（只读，不触发新采样）
 * @param   data  输出数据
 * @retval  0=成功（数据有效）  <0=无可用数据
 */
int app_sensor_get_data(app_sensor_data_t *data);

/**
 * @brief   获取温度字符串（用于调试/日志/上报）
 * @retval  温度字符串，如 "25.50" 或 "--"（无效时）
 * @note    内部使用静态缓冲区，非线程安全（裸机环境可接受）
 */
const char *app_sensor_get_str(void);

/**
 * @brief   获取原始温度值（×0.01°C）
 * @retval  原始值，0 表示无效
 */
int16_t app_sensor_get_raw(void);

/* ========== 回调：温度更新通知（可选） ========== */

/**
 * @brief   注册温度更新回调（FSM/通信模块可注册）
 * @param   cb    回调函数指针
 * @note    当有新温度数据时调用
 */
typedef void (*app_sensor_cb_t)(int16_t temperature);
void app_sensor_register_callback(app_sensor_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_H */
