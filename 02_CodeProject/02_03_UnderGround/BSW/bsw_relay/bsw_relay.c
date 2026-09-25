/**
 * @file    bsw_relay.c
 * @brief   继电器驱动 - BSW 层实现
 * @note    通过 mcal_gpio 控制 PA12，软件层不感知有效电平
 */

#include "bsw_relay.h"
#include "mcal_gpio.h"
#include "bsw_log.h"

/* ========== 私有状态 ========== */
static uint8_t               s_inited       = 0;
static bsw_relay_active_t    s_active_level = BSW_RELAY_ACTIVE_HIGH;
static uint8_t               s_state        = 0;    /* 0=断开，1=吸合 */

/* ========== 内部辅助 ========== */

/**
 * @brief   把"逻辑状态"翻译成 GPIO 实际电平（依有效电平）
 */
static uint8_t _to_gpio_level(uint8_t on)
{
    if (s_active_level == BSW_RELAY_ACTIVE_LOW) {
        return on ? GPIO_LOW : GPIO_HIGH;
    }
    /* ACTIVE_HIGH（默认） */
    return on ? GPIO_HIGH : GPIO_LOW;
}

/* ================================================================ */
/*                           公共接口实现                            */
/* ================================================================ */

bsw_relay_ret_t bsw_relay_init(bsw_relay_active_t active, uint8_t default_state)
{
    if (active != BSW_RELAY_ACTIVE_HIGH && active != BSW_RELAY_ACTIVE_LOW) {
        return BSW_RELAY_ERR_PARAM;
    }

    s_active_level = active;
    s_state        = default_state ? 1 : 0;

    /* 应用默认状态到 GPIO */
    mcal_gpio_write(GPIO_PIN_RELAY, _to_gpio_level(s_state));

    s_inited = 1;

    bsw_log("[RELAY] init OK, active=%s, state=%s\r\n",
            (active == BSW_RELAY_ACTIVE_HIGH) ? "HIGH" : "LOW",
            s_state ? "ON" : "OFF");

    return BSW_RELAY_OK;
}

bsw_relay_ret_t bsw_relay_on(void)
{
    if (!s_inited) {
        return BSW_RELAY_ERR_PARAM;
    }
    if (s_state) return BSW_RELAY_OK;   /* 已在吸合状态，幂等 */

    s_state = 1;
    mcal_gpio_write(GPIO_PIN_RELAY, _to_gpio_level(1));
    bsw_log("[RELAY] ON\r\n");
    return BSW_RELAY_OK;
}

bsw_relay_ret_t bsw_relay_off(void)
{
    if (!s_inited) {
        return BSW_RELAY_ERR_PARAM;
    }
    if (!s_state) return BSW_RELAY_OK;  /* 已在断开状态，幂等 */

    s_state = 0;
    mcal_gpio_write(GPIO_PIN_RELAY, _to_gpio_level(0));
    bsw_log("[RELAY] OFF\r\n");
    return BSW_RELAY_OK;
}

bsw_relay_ret_t bsw_relay_set(uint8_t on)
{
    return on ? bsw_relay_on() : bsw_relay_off();
}

bsw_relay_ret_t bsw_relay_toggle(void)
{
    if (!s_inited) {
        return BSW_RELAY_ERR_PARAM;
    }
    return bsw_relay_set(!s_state);
}

uint8_t bsw_relay_get(void)
{
    return s_state;
}
