/**
 * @file    bsw_led.h
 * @brief   LED 状态指示 - BSW 层
 * @note    5 个 LED 独立指示不同状态：
 *          - LED1 (PB5)：扫频监听态（SCAN_LISTEN）常亮
 *          - LED2 (PB4)：预链接态（PRE_LINKED）常亮
 *          - LED3 (PB3)：工作态（LINKED）常亮
 *          - LED4 (PD2)：扫频模式（SCAN）常亮
 *          - LED5 (PC12)：故障模式（FAULT）常亮
 *          - 其他状态（BOOT/SLEEP）：全灭
 */

#ifndef BSW_LED_H
#define BSW_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== LED 状态定义（对应 FSM 状态） ========== */
typedef enum {
    LED_STATE_ALL_OFF = 0,     /**< 所有 LED 灭（BOOT/SLEEP）*/
    LED_STATE_SCAN,            /**< LED4 亮（SCAN：扫频）*/
    LED_STATE_LISTEN,          /**< LED1 亮（SCAN_LISTEN：扫频监听）*/
    LED_STATE_PRELINK,         /**< LED2 亮（PRE_LINKED：预链接）*/
    LED_STATE_LINKED,          /**< LED3 亮（LINKED：工作）*/
    LED_STATE_FAULT,           /**< LED5 亮（FAULT：故障）*/
} bsw_led_state_t;

/* ========== 函数声明 ========== */

/**
 * @brief   LED 模块初始化
 * @note    初始化 5 个 LED GPIO，全部设为灭
 */
void bsw_led_init(void);

/**
 * @brief   设置 LED 状态
 * @param   state  LED_STATE_ALL_OFF / SCAN / LISTEN / PRELINK / LINKED / FAULT
 * @note    立即切换到新状态（5 个 LED 中只有 1 个亮或全灭）
 */
void bsw_led_set_state(bsw_led_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* BSW_LED_H */
