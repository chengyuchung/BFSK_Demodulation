/**
 * @file    bsw_bmp280.h
 * @brief   BMP280 气压/温度传感器 - BSW 层
 * @note    基于 mcal_i2c 实现 Bosch BMP280 协议：
 *          - 7-bit I2C 地址（0x76 SDO=GND，0x77 SDO=VDD）
 *          - 温度 (×0.01°C) + 压力 (Pa) 物理量输出
 *          - Bosch 官方补偿算法（来自数据手册 §4.2.3）
 *
 *          BSW 层额外提供：
 *          - 设备状态机（UNINIT / OK / ERR）
 *          - 最近一次有效数据缓存
 *          - 时间戳注入（与平台解耦）
 *
 * @note    物理量不做范围校验：井下温压可远超商业级（1100 hPa / 85°C），
 *          采集到什么数据就直接输出，由 App 层自行判定有效性。
 *
 * @dependency  mcal_i2c  (HAL I2C 字节/缓冲区原语)
 *
 * @note        时间戳来源由上层注入（bsw_bmp280_set_tick_source），
 *              本模块不直接调 HAL / FreeRTOS，保持 BSW 层与平台无关。
 */

#ifndef BSW_BMP280_H
#define BSW_BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 硬件配置常量 ========== */
#define BSW_BMP280_I2C_ADDR_0x76        0x76U   /* SDO 拉低 */
#define BSW_BMP280_I2C_ADDR_0x77        0x77U   /* SDO 拉高 */
#define BSW_BMP280_CHIP_ID_EXPECTED     0x58U   /* BMP280 芯片 ID */

/* ========== 返回码定义 ========== */
typedef enum {
    BSW_BMP280_OK              =  0,
    BSW_BMP280_ERR_PARAM       = -1,   /* 参数错误 / NULL 指针 */
    BSW_BMP280_ERR_I2C         = -2,   /* I2C 读写失败 */
    BSW_BMP280_ERR_CHIP_ID     = -3,   /* 芯片 ID 不匹配 (非 0x58) */
    BSW_BMP280_ERR_NOT_INIT    = -4,   /* 未初始化就调用 read */
} bsw_bmp280_ret_t;

/* ========== 状态定义 ========== */
typedef enum {
    BSW_BMP280_STATE_UNINIT = 0,   /* 未初始化 */
    BSW_BMP280_STATE_OK     = 1,   /* 在线（最近一次读成功） */
    BSW_BMP280_STATE_ERR    = 2,   /* 离线（连续失败次数超阈） */
} bsw_bmp280_state_t;

/* ========== 数据结构 ==========
 * 温度单位：×0.01°C（如 2534 表示 25.34°C）
 * 压力单位：Pa     （如 101325 表示 1013.25 hPa）
 */
typedef struct {
    int32_t             temperature_centi_c;
    uint32_t            pressure_pa;
    uint32_t            timestamp;          /* 采样时刻（毫秒，单调递增） */
    bsw_bmp280_state_t  state;
} bsw_bmp280_data_t;

/* ========== 时间戳注入接口 ==========
 * BSW 层需要时间戳做采样戳记，但本身不应直接调 HAL / FreeRTOS。
 * 由上层 (通常是 App 层) 在 bsw_bmp280_init() 之前注入。
 *
 * 示例:
 *   bsw_bmp280_set_tick_source(HAL_GetTick);
 *
 * 注意：若不注入或传 NULL，缓存中的 timestamp 字段将保持为 0。
 */
typedef uint32_t (*bsw_bmp280_tick_fn)(void);
void bsw_bmp280_set_tick_source(bsw_bmp280_tick_fn fn);

/* ========== 函数声明 ========== */

/**
 * @brief   BMP280 BSW 层初始化
 * @param   i2c_id        mcal_i2c 总线号 (如 I2C_ID_1)
 * @param   dev_addr_7bit 7-bit I2C 设备地址 (0x76 或 0x77)
 * @retval  BSW_BMP280_OK            成功
 *          BSW_BMP280_ERR_I2C       I2C 通信失败
 *          BSW_BMP280_ERR_CHIP_ID   芯片 ID 不匹配
 *
 * @note    前置条件：mcal_i2c_init() 必须先完成
 */
bsw_bmp280_ret_t bsw_bmp280_init(uint8_t i2c_id, uint8_t dev_addr_7bit);

/**
 * @brief   读取温度 + 压力（触发一次完整测量）
 * @param   out  输出数据结构
 * @retval  BSW_BMP280_OK            成功
 *          BSW_BMP280_ERR_PARAM     out 为 NULL
 *          BSW_BMP280_ERR_NOT_INIT  未初始化
 *
 * @note    不校验物理量范围：井下温压可远超商业级，由 App 层判定有效性。
 *          失败仅在芯片未初始化时返回；运行时采集到的数据直接输出。
 */
bsw_bmp280_ret_t bsw_bmp280_read(bsw_bmp280_data_t *out);

/**
 * @brief   获取最近一次有效数据（不触发新测量）
 * @param   out  输出数据结构
 * @retval  BSW_BMP280_OK            成功
 *          BSW_BMP280_ERR_PARAM     out 为 NULL
 *          BSW_BMP280_ERR_NOT_INIT  缓存为空（从未成功读过一次）
 */
bsw_bmp280_ret_t bsw_bmp280_get_cached(bsw_bmp280_data_t *out);

/**
 * @brief   检查设备是否在线
 * @retval  1=在线  0=离线/未初始化
 */
int bsw_bmp280_is_online(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_BMP280_H */
