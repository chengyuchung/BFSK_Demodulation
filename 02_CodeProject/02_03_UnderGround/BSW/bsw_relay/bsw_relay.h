/**
 * @file    bsw_relay.h
 * @brief   继电器驱动 - BSW 层
 *
 * @note    本模块用于控制一路 5V 电源开关（继电器/光电开关/MOSFET）。
 *          控制引脚：PA12（CubeMX 标签 Relay_Control，映射 GPIO_PIN_RELAY）
 *
 * 硬件说明：
 *   - STM32L4 GPIO 输出 3.3V 逻辑，但线圈/负载通常需要 5V。
 *     硬件上一般采用：
 *       (a) NPN 三极管（S8050/2N2222）低边驱动：PA12=HIGH → 线圈通电 → active-HIGH
 *       (b) PNP 三极管 / P-MOSFET 高边驱动：   PA12=LOW  → 线圈通电 → active-LOW
 *       (c) 光耦 + 驱动芯片：                       视光耦方向而定
 *     BSW 层把"哪种电平 = 吸合"封装在有效电平配置里，应用层只需 on/off。
 *
 *   - 上电/复位时 PA12 由 CubeMX 默认为 LOW（推挽输出）。
 *     硬件若采用 active-HIGH，则上电默认 = 继电器断开（安全状态）；
 *     若采用 active-LOW，则上电默认 = 继电器吸合（不安全）。
 *     此时应在 init() 中传入 default_state = 0 显式关闭。
 *
 * @dependency  mcal_gpio  (GPIO_PIN_RELAY = PA12)
 *              bsw_log    (init/状态切换日志)
 *
 * @usage
 *   bsw_relay_init(BSW_RELAY_ACTIVE_HIGH, 0);   // 默认关闭，高电平吸合
 *   bsw_relay_on();                               // 5V 母线上电
 *   // ... 外设使用 ...
 *   bsw_relay_off();                              // 5V 母线下电
 *   bsw_relay_get();                              // 查询当前状态
 */

#ifndef BSW_RELAY_H
#define BSW_RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 有效电平（吸合时 GPIO 的电平） ========== */
typedef enum {
    BSW_RELAY_ACTIVE_HIGH = 0,   /* 高电平吸合：NPN 低边驱动（最常见） */
    BSW_RELAY_ACTIVE_LOW  = 1,   /* 低电平吸合：PNP / P-MOSFET 高边驱动 */
} bsw_relay_active_t;

/* ========== 返回码 ========== */
typedef enum {
    BSW_RELAY_OK        =  0,
    BSW_RELAY_ERR_PARAM = -1,    /* 参数越界 / 未初始化 */
} bsw_relay_ret_t;

/* ========== 函数声明 ========== */

/**
 * @brief   初始化继电器控制
 * @param   active        BSW_RELAY_ACTIVE_HIGH / ACTIVE_LOW（依硬件决定）
 * @param   default_state 初始状态：0=OFF（推荐，上电安全）；1=ON
 * @retval  BSW_RELAY_OK
 *          BSW_RELAY_ERR_PARAM
 *
 * @note    调用前需先 mcal_gpio_init()。
 *          本函数幂等：重复调用会用新配置重新设置默认状态。
 */
bsw_relay_ret_t bsw_relay_init(bsw_relay_active_t active, uint8_t default_state);

/**
 * @brief   吸合继电器（5V 母线上电）
 * @retval  BSW_RELAY_OK / ERR_PARAM（未初始化）
 */
bsw_relay_ret_t bsw_relay_on(void);

/**
 * @brief   断开继电器（5V 母线下电）
 * @retval  BSW_RELAY_OK / ERR_PARAM（未初始化）
 */
bsw_relay_ret_t bsw_relay_off(void);

/**
 * @brief   设定继电器状态
 * @param   on  1=吸合，0=断开
 * @retval  BSW_RELAY_OK / ERR_PARAM
 */
bsw_relay_ret_t bsw_relay_set(uint8_t on);

/**
 * @brief   翻转继电器当前状态
 * @retval  BSW_RELAY_OK / ERR_PARAM
 */
bsw_relay_ret_t bsw_relay_toggle(void);

/**
 * @brief   查询继电器当前状态（来自 BSW 内部记录，不回读 GPIO）
 * @retval  1=吸合，0=断开；未初始化返回 0
 */
uint8_t bsw_relay_get(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_RELAY_H */
