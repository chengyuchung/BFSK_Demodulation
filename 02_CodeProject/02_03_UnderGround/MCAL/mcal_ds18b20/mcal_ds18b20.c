/**
 * @file    mcal_ds18b20.c
 * @brief   DS18B20 1-Wire 温度传感器驱动 - MCAL 层实现
 *
 * 关键特性：
 *   - 开漏输出（避免总线冲突）
 *   - 9 字节 scratchpad 读取 + CRC8 校验
 *   - 转换完成轮询（避免固定等待的浪费或不足）
 *   - 断线/全 0xFF/全 0x00 检测
 */

#include "mcal_ds18b20.h"
#include "mcal_gpio.h"
#include "mcal_timer.h"

/* ========== 1-Wire 时序参数 ========== */
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

#define DS18B20_SCRATCHPAD_LEN    9
#define DS18B20_CONVERT_TIMEOUT   800   /* 12bit 转换上限 750ms */

/* ========== Maxim/Dallas CRC8 ========== */
/* 多项式 X^8+X^5+X^4+1 (0x8C 反射) */
static uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (uint8_t)((crc ^ inbyte) & 0x01u);
            crc >>= 1;
            if (mix) crc ^= 0x8Cu;
            inbyte >>= 1;
        }
    }
    return crc;
}

/* ========== 1-Wire 总线操作 ========== */

int mcal_ds18b20_onewire_reset(void)
{
    int presence;
    mcal_gpio_set_output_od(GPIO_PIN_DS18B20);  /* 开漏 */
    mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
    mcal_timer_delay_us(OW_DELAY_US_RESET);
    mcal_gpio_set_input(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(OW_DELAY_US_PRESENCE);
    presence = mcal_gpio_read(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(OW_DELAY_US_RECOVERY);
    return presence;
}

void mcal_ds18b20_onewire_write_bit(uint8_t bit)
{
    if (bit) {
        mcal_gpio_set_output_od(GPIO_PIN_DS18B20);
        mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
        mcal_timer_delay_us(OW_DELAY_US_WRITE1_L);
        mcal_gpio_set_input(GPIO_PIN_DS18B20);     /* 释放总线 */
        mcal_timer_delay_us(OW_DELAY_US_WRITE1_H);
    } else {
        mcal_gpio_set_output_od(GPIO_PIN_DS18B20);
        mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
        mcal_timer_delay_us(OW_DELAY_US_WRITE0_L);
        mcal_gpio_set_input(GPIO_PIN_DS18B20);
        mcal_timer_delay_us(OW_DELAY_US_WRITE0_H);
    }
}

uint8_t mcal_ds18b20_onewire_read_bit(void)
{
    uint8_t bit;
    mcal_gpio_set_output_od(GPIO_PIN_DS18B20);
    mcal_gpio_write(GPIO_PIN_DS18B20, GPIO_LOW);
    mcal_timer_delay_us(OW_DELAY_US_READ_INIT);
    mcal_gpio_set_input(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(OW_DELAY_US_READ_SAMP);
    bit = mcal_gpio_read(GPIO_PIN_DS18B20);
    mcal_timer_delay_us(OW_DELAY_US_READ_END);
    return bit;
}

void mcal_ds18b20_onewire_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        mcal_ds18b20_onewire_write_bit(data & 0x01u);
        data >>= 1;
    }
}

uint8_t mcal_ds18b20_onewire_read_byte(void)
{
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++) {
        data >>= 1;
        if (mcal_ds18b20_onewire_read_bit()) {
            data |= 0x80u;
        }
    }
    return data;
}

/* ========== DS18B20 命令接口 ========== */

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

/**
 * @brief   轮询等待温度转换完成
 * @note    转换期间 DS18B20 拉低 DQ，完成后释放（变高）
 *          用 read_bit 采样 DQ 状态判断
 */
ds18b20_ret_t mcal_ds18b20_wait_conversion(uint32_t timeout_ms)
{
    uint32_t waited = 0;

    /* 起始稍等片刻，避免在转换刚开始就采样导致误判 */
    mcal_timer_delay_us(10);

    while (waited < timeout_ms) {
        /* read_bit 内部拉低 6us + 释放 + 在 15us 处采样 */
        if (mcal_ds18b20_onewire_read_bit() != 0) {
            return DS18B20_OK;   /* 转换完成（DQ 被释放=高） */
        }
        mcal_timer_delay_ms(10);
        waited += 10;
    }

    return DS18B20_ERR_TIMEOUT;
}

ds18b20_ret_t mcal_ds18b20_read_result(int16_t *temp_out)
{
    uint8_t scratch[DS18B20_SCRATCHPAD_LEN];
    uint8_t crc_calc;
    int16_t raw;

    if (temp_out == NULL) return DS18B20_ERR_READ;

    if (mcal_ds18b20_onewire_reset() != 0) return DS18B20_ERR_NO_DEVICE;

    mcal_ds18b20_onewire_write_byte(0xCC); /* Skip ROM */
    mcal_ds18b20_onewire_write_byte(0xBE); /* Read Scratchpad */

    for (uint8_t i = 0; i < DS18B20_SCRATCHPAD_LEN; i++) {
        scratch[i] = mcal_ds18b20_onewire_read_byte();
    }

    /* 断线常读出全 0xFF 或全 0x00，提前识别 */
    uint8_t all_ff = 1, all_00 = 1;
    for (uint8_t i = 0; i < DS18B20_SCRATCHPAD_LEN; i++) {
        if (scratch[i] != 0xFF) all_ff = 0;
        if (scratch[i] != 0x00) all_00 = 0;
    }
    if (all_ff || all_00) return DS18B20_ERR_READ;

    /* CRC8 校验 */
    crc_calc = ds18b20_crc8(scratch, 8);
    if (crc_calc != scratch[8]) return DS18B20_ERR_CRC;

    /* LSB=温度低字节，MSB=温度高字节（符号扩展） */
    raw = (int16_t)((uint16_t)scratch[1] << 8) | scratch[0];

    /* 12bit 分辨率：1 LSB = 1/16 °C → 换算成 ×100 整数 */
    *temp_out = (int16_t)((int32_t)raw * 100 / 16);

    return DS18B20_OK;
}

ds18b20_ret_t mcal_ds18b20_read_temp(int16_t *temp_out)
{
    ds18b20_ret_t ret;

    if (temp_out == NULL) return DS18B20_ERR_READ;

    ret = mcal_ds18b20_start_convert();
    if (ret != DS18B20_OK) return ret;

    /* 12bit 默认分辨率，最长 750ms */
    ret = mcal_ds18b20_wait_conversion(DS18B20_CONVERT_TIMEOUT);
    if (ret != DS18B20_OK) return ret;

    return mcal_ds18b20_read_result(temp_out);
}
