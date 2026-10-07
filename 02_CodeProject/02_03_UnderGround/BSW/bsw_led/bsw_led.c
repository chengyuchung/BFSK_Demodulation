/**
 * @file    bsw_led.c
 * @brief   LED 状态指示 - BSW 层实现
 * @note    5 个 LED 独立控制，纯 GPIO 输出，无需定时器中断
 */

#include "bsw_led.h"
#include "mcal_gpio.h"

/* ========== 内部状态 ========== */
static bsw_led_state_t s_current_state = LED_STATE_ALL_OFF;

/* ========== 公共接口实现 ========== */

void bsw_led_init(void)
{
    /* GPIO 初始化由 mcal_gpio_init() 完成 */
    /* 这里只设置初始状态：所有 LED 灭 */
    s_current_state = LED_STATE_ALL_OFF;
    
    mcal_gpio_write(GPIO_PIN_LED_LISTEN,  GPIO_LOW);
    mcal_gpio_write(GPIO_PIN_LED_PRELINK, GPIO_LOW);
    mcal_gpio_write(GPIO_PIN_LED_LINK,    GPIO_LOW);
    mcal_gpio_write(GPIO_PIN_LED_SCAN,    GPIO_LOW);
    mcal_gpio_write(GPIO_PIN_LED_ERROR,   GPIO_LOW);
}

void bsw_led_set_state(bsw_led_state_t state)
{
    if (s_current_state == state) {
        return;  /* 状态未变化，避免重复操作 */
    }
    
    s_current_state = state;
    
    /* 根据状态设置 5 个 LED（同时只有 1 个亮或全灭）*/
    switch (state) {
        case LED_STATE_SCAN:
            /* 扫频模式：LED4 亮，其他灭 */
            mcal_gpio_write(GPIO_PIN_LED_LISTEN,  GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_PRELINK, GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_LINK,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_SCAN,    GPIO_HIGH);
            mcal_gpio_write(GPIO_PIN_LED_ERROR,   GPIO_LOW);
            break;
            
        case LED_STATE_LISTEN:
            /* 扫频监听态：LED1 亮，其他灭 */
            mcal_gpio_write(GPIO_PIN_LED_LISTEN,  GPIO_HIGH);
            mcal_gpio_write(GPIO_PIN_LED_PRELINK, GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_LINK,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_SCAN,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_ERROR,   GPIO_LOW);
            break;
            
        case LED_STATE_PRELINK:
            /* 预链接态：LED2 亮，其他灭 */
            mcal_gpio_write(GPIO_PIN_LED_LISTEN,  GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_PRELINK, GPIO_HIGH);
            mcal_gpio_write(GPIO_PIN_LED_LINK,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_SCAN,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_ERROR,   GPIO_LOW);
            break;
            
        case LED_STATE_LINKED:
            /* 工作态：LED3 亮，其他灭 */
            mcal_gpio_write(GPIO_PIN_LED_LISTEN,  GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_PRELINK, GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_LINK,    GPIO_HIGH);
            mcal_gpio_write(GPIO_PIN_LED_SCAN,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_ERROR,   GPIO_LOW);
            break;
            
        case LED_STATE_FAULT:
            /* 故障模式：LED5 亮，其他灭 */
            mcal_gpio_write(GPIO_PIN_LED_LISTEN,  GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_PRELINK, GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_LINK,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_SCAN,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_ERROR,   GPIO_HIGH);
            break;
            
        case LED_STATE_ALL_OFF:
        default:
            /* 其他状态（BOOT/SLEEP）：全灭 */
            mcal_gpio_write(GPIO_PIN_LED_LISTEN,  GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_PRELINK, GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_LINK,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_SCAN,    GPIO_LOW);
            mcal_gpio_write(GPIO_PIN_LED_ERROR,   GPIO_LOW);
            break;
    }
}
