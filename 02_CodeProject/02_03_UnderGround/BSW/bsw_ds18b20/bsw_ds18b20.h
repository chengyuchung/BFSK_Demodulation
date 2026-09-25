/**
 * @file    bsw_ds18b20.h
 * @brief   DS18B20 温度传感器 - BSW 层
 * @note    基于 mcal_ow（1-Wire 时序）实现 DS18B20 协议：
 *          - Skip ROM 命令
 *          - Convert T / Read Scratchpad 命令
 *          - CRC8 校验
 *          - 12bit 温度换算 (×0.01°C)
 *
 *          BSW 层额外提供：
 *          - 设备在线状态机
 *          - 最近一次有效温度缓存
 *          - 连续失败计数 → 自动标记离线
 *
 * @dependency  mcal_ow (1-Wire 时序)
 * @dependency  mcal_timer (us/ms 延时)
 *
 * @note        时间戳来源由上层注入（bsw_ds18b20_set_tick_source），
 *              本模块不直接调 HAL / FreeRTOS，保持 BSW 层与平台无关。
 */

#ifndef BSW_DS18B20_H
#define BSW_DS18B20_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 返回码定义（DS18B20 协议层） ========== */
typedef enum {
    DS18B20_OK             =  0,
    DS18B20_ERR_NO_DEVICE  = -1,   /* 总线无设备响应 */
    DS18B20_ERR_CRC        = -2,   /* CRC8 校验失败 */
    DS18B20_ERR_TIMEOUT    = -3,   /* 转换超时 */
    DS18B20_ERR_READ       = -4,   /* 读取异常（断线/全 0xFF/全 0x00） */
} ds18b20_ret_t;

/* ========== 状态定义（BSW 抽象） ========== */
typedef enum {
    BSW_DS18B20_UNINIT = 0,   /* 未初始化 */
    BSW_DS18B20_OK     = 1,   /* 在线 */
    BSW_DS18B20_ERR    = 2,   /* 离线（连续失败次数超阈） */
} bsw_ds18b20_state_t;

/* ========== 数据结构 ========== */
typedef struct {
    int16_t             raw_value;   /* 原始温度（×0.01°C），如 2550 = 25.50°C */
    uint32_t            timestamp;   /* 采样时刻（毫秒，单调递增，来源由上层注入） */
    bsw_ds18b20_state_t state;       /* 传感器状态 */
} bsw_ds18b20_data_t;

/* ========== 时间戳注入接口 ==========
 * BSW 层需要时间戳做采样戳记，但本身不应直接调 HAL / FreeRTOS。
 * 由上层 (通常是 App 层) 在 bsw_ds18b20_init() 之前注入。
 *
 * 示例:
 *   bsw_ds18b20_set_tick_source(HAL_GetTick);    // App 层调 HAL 是 CubeMX 约定
 *   bsw_ds18b20_set_tick_source(xTaskGetTickCount);  // 或 RTOS 接口
 *
 * 注意：若不注入，缓存中的 timestamp 字段将保持为 0。
 */
typedef uint32_t (*bsw_ds18b20_tick_fn)(void);
void bsw_ds18b20_set_tick_source(bsw_ds18b20_tick_fn fn);

/* ========== 函数声明 ========== */

/**
 * @brief   DS18B20 BSW 层初始化（内部触发 1-Wire 复位检测）
 * @retval  0=成功  <0=错误码（参见 ds18b20_ret_t）
 */
int bsw_ds18b20_init(void);

/**
 * @brief   启动一次温度转换（非阻塞）
 * @retval  0=成功  <0=错误码
 */
int bsw_ds18b20_trigger(void);

/**
 * @brief   读取温度结果（内部含 wait_conversion + read_scratchpad + CRC）
 * @param   data  输出温度数据
 * @retval  0=成功  <0=错误码
 */
int bsw_ds18b20_get_result(bsw_ds18b20_data_t *data);

/**
 * @brief   获取最近一次有效温度（不触发新转换）
 * @param   data  输出温度数据
 * @retval  0=成功  <0=无有效数据
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
