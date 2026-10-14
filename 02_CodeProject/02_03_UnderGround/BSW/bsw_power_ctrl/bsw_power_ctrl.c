/**
 * @file    bsw_power_ctrl.c
 * @brief   电源控制模块 - BSW 层实现
 * @note    多通道电源开关控制，支持继电器、功放等
 */

#include "bsw_power_ctrl.h"
#include "mcal_gpio.h"

/* ========== 通道配置表项 ========== */
typedef struct {
    uint8_t               gpio_pin;       /* GPIO 引脚编号 */
    power_active_level_t  active_level;   /* 有效电平 */
    uint8_t               state;          /* 当前状态：0=关闭，1=开启 */
} power_channel_config_t;

/* ========== 私有状态 ========== */

/**
 * @brief 模块初始化标志
 * @rationale: 防止未初始化就使用，必须是静态
 * @note: 仅在 bsw_power_ctrl_init() 中设置
 */
static uint8_t s_inited = 0;

/**
 * @brief 通道配置表
 * @rationale: 集中管理所有电源通道配置，添加新通道只需在此表中添加一项
 * @note: 索引与 power_channel_t 枚举值对应
 * 
 * @details
 *  每个通道配置：
 *  - gpio_pin: 对应 mcal_gpio.h 中的 GPIO_PIN_xxx
 *  - active_level: ACTIVE_HIGH（高电平导通）或 ACTIVE_LOW（低电平导通）
 *  - state: 运行时状态，初始化时设为 0（关闭）
 */
static power_channel_config_t s_channel_config[POWER_CH_COUNT] = {
    /* POWER_CH_RELAY: 继电器（5V 母线） */
    {
        .gpio_pin     = GPIO_PIN_RELAY,
        .active_level = POWER_ACTIVE_HIGH,   /* 高电平吸合（NPN 驱动） */
        .state        = 0,                   /* 初始关闭 */
    },
    
    /* POWER_CH_PA: 功率放大器电源 */
    {
        .gpio_pin     = GPIO_PIN_PA_POWER,
        .active_level = POWER_ACTIVE_HIGH,   /* 高电平导通 */
        .state        = 0,                   /* 初始关闭 */
    },
};

/* ========== 内部辅助函数 ========== */

/**
 * @brief   将逻辑状态转换为 GPIO 实际电平
 * @param   p_config  通道配置指针
 * @param   on        逻辑状态：1=开启，0=关闭
 * @return  uint8_t   GPIO 电平：GPIO_HIGH / GPIO_LOW
 * 
 * @details
 *  为什么需要转换：
 *  - 应用层只关心"开/关"逻辑
 *  - 硬件驱动电路决定了实际电平（ACTIVE_HIGH 或 ACTIVE_LOW）
 *  - 本函数封装转换逻辑，应用层无需感知硬件细节
 */
static uint8_t power_ctrl_to_gpio_level(const power_channel_config_t *p_config, uint8_t on)
{
    if (p_config->active_level == POWER_ACTIVE_LOW)
    {
        /* 低电平有效：开启=LOW，关闭=HIGH */
        return on ? GPIO_LOW : GPIO_HIGH;
    }
    
    /* 高电平有效（默认）：开启=HIGH，关闭=LOW */
    return on ? GPIO_HIGH : GPIO_LOW;
}

/**
 * @brief   参数检查：通道编号是否有效
 * @param   ch  通道编号
 * @return  uint8_t  1=有效，0=无效
 */
static uint8_t power_ctrl_is_valid_channel(power_channel_t ch)
{
    return (ch < POWER_CH_COUNT);
}

/* ================================================================ */
/*                           公共接口实现                            */
/* ================================================================ */

void bsw_power_ctrl_init(void)
{
    /* 为什么：遍历所有通道，设置初始状态为关闭 */
    for (uint8_t ch = 0; ch < POWER_CH_COUNT; ch++)
    {
        power_channel_config_t *p_config = &s_channel_config[ch];
        
        /* 设置初始状态为关闭 */
        p_config->state = 0;
        
        /* 将关闭状态写入 GPIO */
        uint8_t gpio_level = power_ctrl_to_gpio_level(p_config, 0);
        mcal_gpio_write(p_config->gpio_pin, gpio_level);
    }
    
    /* 标记模块已初始化 */
    s_inited = 1;
}

power_ctrl_ret_t bsw_power_ctrl_on(power_channel_t ch)
{
    /* 参数检查 */
    if (!power_ctrl_is_valid_channel(ch))
    {
        return POWER_CTRL_ERR_CH;
    }
    
    if (!s_inited)
    {
        return POWER_CTRL_ERR_INIT;
    }
    
    power_channel_config_t *p_config = &s_channel_config[ch];
    
    /* 为什么：幂等操作，如果已开启则直接返回成功 */
    if (p_config->state == 1)
    {
        return POWER_CTRL_OK;
    }
    
    /* 更新状态 */
    p_config->state = 1;
    
    /* 写入 GPIO */
    uint8_t gpio_level = power_ctrl_to_gpio_level(p_config, 1);
    mcal_gpio_write(p_config->gpio_pin, gpio_level);
    
    return POWER_CTRL_OK;
}

power_ctrl_ret_t bsw_power_ctrl_off(power_channel_t ch)
{
    /* 参数检查 */
    if (!power_ctrl_is_valid_channel(ch))
    {
        return POWER_CTRL_ERR_CH;
    }
    
    if (!s_inited)
    {
        return POWER_CTRL_ERR_INIT;
    }
    
    power_channel_config_t *p_config = &s_channel_config[ch];
    
    /* 为什么：幂等操作，如果已关闭则直接返回成功 */
    if (p_config->state == 0)
    {
        return POWER_CTRL_OK;
    }
    
    /* 更新状态 */
    p_config->state = 0;
    
    /* 写入 GPIO */
    uint8_t gpio_level = power_ctrl_to_gpio_level(p_config, 0);
    mcal_gpio_write(p_config->gpio_pin, gpio_level);
    
    return POWER_CTRL_OK;
}

power_ctrl_ret_t bsw_power_ctrl_set(power_channel_t ch, uint8_t on)
{
    /* 为什么：统一入口，根据 on 值调用 on() 或 off() */
    return on ? bsw_power_ctrl_on(ch) : bsw_power_ctrl_off(ch);
}

power_ctrl_ret_t bsw_power_ctrl_toggle(power_channel_t ch)
{
    /* 参数检查 */
    if (!power_ctrl_is_valid_channel(ch))
    {
        return POWER_CTRL_ERR_CH;
    }
    
    if (!s_inited)
    {
        return POWER_CTRL_ERR_INIT;
    }
    
    /* 为什么：读取当前状态，翻转后设置 */
    uint8_t current_state = s_channel_config[ch].state;
    return bsw_power_ctrl_set(ch, !current_state);
}

uint8_t bsw_power_ctrl_get(power_channel_t ch)
{
    /* 参数检查 */
    if (!power_ctrl_is_valid_channel(ch))
    {
        return 0;
    }
    
    if (!s_inited)
    {
        return 0;
    }
    
    /* 为什么：返回记录的状态，不回读 GPIO（避免硬件访问开销） */
    return s_channel_config[ch].state;
}
