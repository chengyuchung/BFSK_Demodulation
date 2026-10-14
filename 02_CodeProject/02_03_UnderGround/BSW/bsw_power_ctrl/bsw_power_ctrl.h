/**
 * @file    bsw_power_ctrl.h
 * @brief   电源控制模块 - BSW 层
 *
 * @note    本模块统一管理多路电源开关（继电器、功放、传感器供电等）
 *          每个通道独立配置有效电平，应用层只需 on/off 操作
 *
 * 硬件说明：
 *   - 继电器通道（POWER_CH_RELAY）：PA12，控制 5V 母线
 *   - 功放通道（POWER_CH_PA）：PC8，控制功率放大器电源
 *   - 每个通道支持 active-HIGH 或 active-LOW 配置
 *   - 上电默认所有通道关闭（安全状态）
 *
 * @dependency  mcal_gpio  (GPIO 引脚控制)
 *
 * @usage
 *   bsw_power_ctrl_init();                        // 初始化所有通道
 *   bsw_power_ctrl_on(POWER_CH_RELAY);           // 5V 母线上电
 *   bsw_power_ctrl_on(POWER_CH_PA);              // 功放上电
 *   bsw_power_ctrl_off(POWER_CH_PA);             // 功放下电
 *   uint8_t state = bsw_power_ctrl_get(POWER_CH_RELAY);  // 查询状态
 */

#ifndef BSW_POWER_CTRL_H
#define BSW_POWER_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 电源通道定义 ========== */
typedef enum {
    POWER_CH_RELAY = 0,    /* 继电器（5V 母线开关） */
    POWER_CH_PA,           /* 功率放大器电源 */
    POWER_CH_COUNT         /* 通道总数（内部使用） */
} power_channel_t;

/* ========== 有效电平（通道导通时 GPIO 的电平） ========== */
typedef enum {
    POWER_ACTIVE_HIGH = 0,   /* 高电平导通：NPN 低边驱动（常见） */
    POWER_ACTIVE_LOW  = 1,   /* 低电平导通：PNP / P-MOSFET 高边驱动 */
} power_active_level_t;

/* ========== 返回码 ========== */
typedef enum {
    POWER_CTRL_OK        =  0,   /* 操作成功 */
    POWER_CTRL_ERR_CH    = -1,   /* 通道编号越界 */
    POWER_CTRL_ERR_INIT  = -2,   /* 模块未初始化 */
} power_ctrl_ret_t;

/* ========== 函数声明 ========== */

/**
 * @brief   初始化电源控制模块
 * 
 * @details
 *  初始化流程：
 *  1. 配置所有通道的有效电平
 *  2. 将所有通道设置为关闭状态（安全）
 *  3. 标记模块已初始化
 * 
 * @note 调用前需先 mcal_gpio_init()
 *       本函数幂等：重复调用会重新初始化所有通道
 */
void bsw_power_ctrl_init(void);

/**
 * @brief   开启指定电源通道
 * @param   ch  电源通道编号（POWER_CH_RELAY / POWER_CH_PA）
 * @return  power_ctrl_ret_t  POWER_CTRL_OK / ERR_CH / ERR_INIT
 * 
 * @note 幂等操作：如果通道已开启，直接返回成功
 */
power_ctrl_ret_t bsw_power_ctrl_on(power_channel_t ch);

/**
 * @brief   关闭指定电源通道
 * @param   ch  电源通道编号（POWER_CH_RELAY / POWER_CH_PA）
 * @return  power_ctrl_ret_t  POWER_CTRL_OK / ERR_CH / ERR_INIT
 * 
 * @note 幂等操作：如果通道已关闭，直接返回成功
 */
power_ctrl_ret_t bsw_power_ctrl_off(power_channel_t ch);

/**
 * @brief   设置指定电源通道状态
 * @param   ch  电源通道编号
 * @param   on  1=开启，0=关闭
 * @return  power_ctrl_ret_t  POWER_CTRL_OK / ERR_CH / ERR_INIT
 */
power_ctrl_ret_t bsw_power_ctrl_set(power_channel_t ch, uint8_t on);

/**
 * @brief   翻转指定电源通道状态
 * @param   ch  电源通道编号
 * @return  power_ctrl_ret_t  POWER_CTRL_OK / ERR_CH / ERR_INIT
 */
power_ctrl_ret_t bsw_power_ctrl_toggle(power_channel_t ch);

/**
 * @brief   查询指定电源通道状态
 * @param   ch  电源通道编号
 * @return  uint8_t  1=开启，0=关闭；通道无效或未初始化返回 0
 * 
 * @note 返回值来自 BSW 内部记录，不回读 GPIO 实际电平
 */
uint8_t bsw_power_ctrl_get(power_channel_t ch);

/* ========== 兼容层：保留原有 bsw_relay 接口 ========== */

/**
 * @brief   继电器初始化（兼容接口）
 * @param   active        BSW_RELAY_ACTIVE_HIGH / ACTIVE_LOW（依硬件决定）
 * @param   default_state 初始状态：0=OFF（推荐）；1=ON
 * @return  int  0=成功，-1=失败
 * 
 * @note 本函数已废弃，推荐使用 bsw_power_ctrl_init()
 *       为保持向后兼容，仍保留此接口
 */
#define BSW_RELAY_ACTIVE_HIGH  POWER_ACTIVE_HIGH
#define BSW_RELAY_ACTIVE_LOW   POWER_ACTIVE_LOW
#define BSW_RELAY_OK           POWER_CTRL_OK
#define BSW_RELAY_ERR_PARAM    POWER_CTRL_ERR_CH

typedef power_active_level_t bsw_relay_active_t;
typedef power_ctrl_ret_t bsw_relay_ret_t;

static inline bsw_relay_ret_t bsw_relay_init(bsw_relay_active_t active, uint8_t default_state)
{
    (void)active;         /* 忽略，由 power_ctrl 内部配置 */
    (void)default_state;  /* 忽略，默认关闭 */
    bsw_power_ctrl_init();
    return BSW_RELAY_OK;
}

/**
 * @brief   继电器开启（兼容接口）
 */
static inline bsw_relay_ret_t bsw_relay_on(void)
{
    return bsw_power_ctrl_on(POWER_CH_RELAY);
}

/**
 * @brief   继电器关闭（兼容接口）
 */
static inline bsw_relay_ret_t bsw_relay_off(void)
{
    return bsw_power_ctrl_off(POWER_CH_RELAY);
}

/**
 * @brief   设置继电器状态（兼容接口）
 */
static inline bsw_relay_ret_t bsw_relay_set(uint8_t on)
{
    return bsw_power_ctrl_set(POWER_CH_RELAY, on);
}

/**
 * @brief   翻转继电器状态（兼容接口）
 */
static inline bsw_relay_ret_t bsw_relay_toggle(void)
{
    return bsw_power_ctrl_toggle(POWER_CH_RELAY);
}

/**
 * @brief   查询继电器状态（兼容接口）
 */
static inline uint8_t bsw_relay_get(void)
{
    return bsw_power_ctrl_get(POWER_CH_RELAY);
}

#ifdef __cplusplus
}
#endif

#endif /* BSW_POWER_CTRL_H */
