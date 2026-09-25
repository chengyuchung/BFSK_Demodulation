/**
 * @file    mcal_ow.c
 * @brief   1-Wire (单总线) 时序驱动 - MCAL 层实现
 * @note    严格遵循 Maxim/Dallas 1-Wire 时序规范：
 *          - Reset: 主机拉低 ≥480us，采样 15~60us 区段
 *          - Write 1: 拉低 ~6us 后释放
 *          - Write 0: 拉低 ~60us 后释放
 *          - Read:    拉低 ~6us 后释放，在 ~15us 处采样
 *
 *          使用开漏输出模式，避免多设备总线冲突。
 */

#include "mcal_ow.h"
#include "mcal_gpio.h"
#include "mcal_timer.h"

/* ========== 1-Wire 时序参数（单位：微秒） ========== */
#define OW_DELAY_US_RESET     480
#define OW_DELAY_US_PRESENCE  70
#define OW_DELAY_US_RECOVERY  410
#define OW_DELAY_US_WRITE1_L  6
#define OW_DELAY_US_WRITE1_H  64
#define OW_DELAY_US_WRITE0_L  60
#define OW_DELAY_US_WRITE0_H  10
#define OW_DELAY_US_READ_INIT 6
#define OW_DELAY_US_READ_SAMP 9
#define OW_DELAY_US_READ_END  55

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */

ow_ret_t mcal_ow_reset(void)
{
    int presence;

    /* 主机发 reset 脉冲：拉低 480us */
    mcal_gpio_set_output_od(GPIO_PIN_DS18B20);  /* 开漏输出 */
    mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
    mcal_timer_delay_us(OW_DELAY_US_RESET);

    /* 释放总线，等待 presence（DS18B20 会在 15~60us 拉低） */
    mcal_gpio_set_input(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(OW_DELAY_US_PRESENCE);

    presence = mcal_gpio_read(GPIO_PIN_DS18B20);

    /* 等待时隙结束，再处理下一个动作 */
    mcal_timer_delay_us(OW_DELAY_US_RECOVERY);

    return (presence == 0) ? OW_OK : OW_ERR_NO_DEVICE;
}

void mcal_ow_write_bit(uint8_t bit)
{
    mcal_gpio_set_output_od(GPIO_PIN_DS18B20);
    mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);

    if (bit) {
        /* Write 1: 拉低 ~6us 后释放，靠上拉电阻维持高 */
        mcal_timer_delay_us(OW_DELAY_US_WRITE1_L);
        mcal_gpio_set_input(GPIO_PIN_DS18B20);
        mcal_timer_delay_us(OW_DELAY_US_WRITE1_H);
    } else {
        /* Write 0: 拉低 ~60us 全程 */
        mcal_timer_delay_us(OW_DELAY_US_WRITE0_L);
        mcal_gpio_set_input(GPIO_PIN_DS18B20);
        mcal_timer_delay_us(OW_DELAY_US_WRITE0_H);
    }
}

uint8_t mcal_ow_read_bit(void)
{
    uint8_t bit;

    mcal_gpio_set_output_od(GPIO_PIN_DS18B20);
    mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
    mcal_timer_delay_us(OW_DELAY_US_READ_INIT);   /* 拉低 ~6us */

    mcal_gpio_set_input(GPIO_PIN_DS18B20);        /* 释放 */
    mcal_timer_delay_us(OW_DELAY_US_READ_SAMP);   /* ~15us 处采样 */

    bit = mcal_gpio_read(GPIO_PIN_DS18B20);

    mcal_timer_delay_us(OW_DELAY_US_READ_END);     /* 完成 slot */
    return bit;
}

void mcal_ow_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        mcal_ow_write_bit(data & 0x01u);
        data >>= 1;
    }
}

uint8_t mcal_ow_read_byte(void)
{
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++) {
        data >>= 1;
        if (mcal_ow_read_bit()) {
            data |= 0x80u;
        }
    }
    return data;
}
