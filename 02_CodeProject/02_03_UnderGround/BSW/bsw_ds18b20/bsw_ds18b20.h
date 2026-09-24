/**
 * @file    bsw_ds18b20.h
 * @brief   DS18B20 传感器 - BSW 层
 * @note    在 MCAL 层基础上增加：初始化检查 + 温度缓存管理
 *          负责定时启动转换、控制采样周期
 *
 * @dependency  mcal_ds18b20
 */

#ifndef BSW_DS18B20_H
#define BSW_DS18B20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mcal_ds18b20.h"

/* ========== 类型定义 ========== */

/**
 * @brief   DS18B20 状态
 */
typedef enum {
    BSW_DS18B20_UNINIT   = 0,   /* 未初始化 */
    BSW_DS18B20_OK       = 1,   /* 正常 */
    BSW_DS18B20_ERR      = 2,   /* 设备故障 */
} bsw_ds18b20_state_t;

/**
 * @brief   温度数据结构（供 APP 层读取）
 */
typedef struct {
    int16_t   raw_value;        /* 原始温度值（×0.01°C），如 2550 表示 25.50°C */
    uint32_t  timestamp;        /* 采样时刻（HAL_GetTick 单位：ms） */
    bsw_ds18b20_state_t state;  /* 传感器状态 */
} bsw_ds18b20_data_t;

/* ========== 函数声明 ========== */

/**
 * @brief   DS18B20 BSW 层初始化
 * @retval  0=成功  <0=错误码
 * @note    内部调用 mcal_ds18b20_init() 检测设备
 */
int bsw_ds18b20_init(void);

/**
 * @brief   启动一次温度转换（非阻塞，需后续调用 bsw_ds18b20_get_result 读取）
 * @retval  0=成功  <0=错误码
 */
int bsw_ds18b20_trigger(void);

/**
 * @brief   读取温度结果（内部包含最多 80ms 等待）
 * @param   data    输出温度数据
 * @retval  0=成功  <0=错误码
 */
int bsw_ds18b20_get_result(bsw_ds18b20_data_t *data);

/**
 * @brief   获取最近一次有效温度（不触发新转换）
 * @param   data    输出温度数据
 * @retval  0=成功（有缓存数据）  <0=无有效数据
 * @note    APP 层轮询时推荐使用此接口，避免频繁启动转换
 */
int bsw_ds18b20_get_cached(bsw_ds18b20_data_t *data);

/**
 * @brief   检查设备是否在线
 * @retval  1=在线  0=离线
 */
int bsw_ds18b20_is_online(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_DS18B20_H */
