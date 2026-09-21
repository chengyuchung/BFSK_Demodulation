/* mcal_ds18b20.c */
#include "mcal_ds18b20.h"
#include "mcal_gpio.h"
#include "mcal_timer.h"

/* 1-Wire 时序参数 */
#define OW_DELAY_US_500   500
#define OW_DELAY_US_65    65
#define OW_DELAY_US_10    10
#define OW_DELAY_US_55    55

int mcal_ds18b20_onewire_reset(void)
{
    int presence;
    mcal_gpio_set_output(GPIO_PIN_DS18B20);
    mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
    mcal_timer_delay_us(500);
    mcal_gpio_set_input(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(65);
    presence = mcal_gpio_read(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(455);
    return presence;
}

void mcal_ds18b20_onewire_write_bit(uint8_t bit)
{
    if (bit) {
        mcal_gpio_set_output(GPIO_PIN_DS18B20);
        mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
        mcal_timer_delay_us(10);
        mcal_gpio_set_input(GPIO_PIN_DS18B20);
        mcal_timer_delay_us(55);
    } else {
        mcal_gpio_set_output(GPIO_PIN_DS18B20);
        mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
        mcal_timer_delay_us(65);
        mcal_gpio_set_input(GPIO_PIN_DS18B20);
        mcal_timer_delay_us(5);
    }
}

uint8_t mcal_ds18b20_onewire_read_bit(void)
{
    uint8_t bit;
    mcal_gpio_set_output(GPIO_PIN_DS18B20);
    mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
    mcal_timer_delay_us(10);
    mcal_gpio_set_input(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(10);
    bit = mcal_gpio_read(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(55);
    return bit;
}

void mcal_ds18b20_onewire_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        mcal_ds18b20_onewire_write_bit(data & 0x01);
        data >>= 1;
    }
}

uint8_t mcal_ds18b20_onewire_read_byte(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data >>= 1;
        if (mcal_ds18b20_onewire_read_bit()) {
            data |= 0x80;
        }
    }
    return data;
}

ds18b20_ret_t mcal_ds18b20_init(void)
{
    if (mcal_ds18b20_onewire_reset() != 0) {
        return DS18B20_ERR_NO_DEVICE;
    }
    return DS18B20_OK;
}

ds18b20_ret_t mcal_ds18b20_start_convert(void)
{
    if (mcal_ds18b20_onewire_reset() != 0) return DS18B20_ERR_NO_DEVICE;
    mcal_ds18b20_onewire_write_byte(0xCC); /* Skip ROM */
    mcal_ds18b20_onewire_write_byte(0x44); /* Convert T */
    return DS18B20_OK;
}

ds18b20_ret_t mcal_ds18b20_read_result(int16_t *temp_out)
{
    uint8_t LSB, MSB;
    if (mcal_ds18b20_onewire_reset() != 0) return DS18B20_ERR_NO_DEVICE;
    mcal_ds18b20_onewire_write_byte(0xCC); /* Skip ROM */
    mcal_ds18b20_onewire_write_byte(0xBE); /* Read Scratchpad */
    LSB = mcal_ds18b20_onewire_read_byte();
    MSB = mcal_ds18b20_onewire_read_byte();
    int16_t raw = (MSB << 8) | LSB;
    *temp_out = (raw * 100) / 16; /* 扩大100倍 */
    return DS18B20_OK;
}

ds18b20_ret_t mcal_ds18b20_read_temp(int16_t *temp_out)
{
    ds18b20_ret_t ret;
    ret = mcal_ds18b20_start_convert();
    if (ret != DS18B20_OK) return ret;
    mcal_timer_delay_ms(80); /* 等待转换完成 */
    return mcal_ds18b20_read_result(temp_out);
}
