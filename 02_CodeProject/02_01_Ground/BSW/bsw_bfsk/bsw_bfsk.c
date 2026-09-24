/* bsw_bfsk.c */
#include "bsw_bfsk.h"
#include "mcal_spi.h"
#include "mcal_gpio.h"

static bsw_bfsk_config_t g_cfg;

void bsw_bfsk_init(const bsw_bfsk_config_t *cfg)
{
    g_cfg = *cfg;
}

/* 简化 Goertzel：只演示框架，实际参数需按采样率计算 k */
int32_t bsw_goertzel(const int16_t *samples, uint32_t len, uint32_t freq, uint32_t fs)
{
    (void)samples; (void)len; (void)freq; (void)fs;
    /* TODO: 实现 Goertzel 算法
     * k = round(N * freq / fs)
     * 迭代计算 Q1, Q2, Q0
     * 幅值 = sqrt(Q1^2 + Q2^2 - Q1*Q2*coeff)
     */
    return 0;
}

bsfk_bit_t bsw_bfsk_demod_bit(const int16_t *samples, uint32_t len)
{
    int32_t mag0 = bsw_goertzel(samples, len, g_cfg.f0, g_cfg.sample_rate);
    int32_t mag1 = bsw_goertzel(samples, len, g_cfg.f1, g_cfg.sample_rate);
    if (mag0 > mag1) return BFSK_BIT_0;
    if (mag1 > mag0) return BFSK_BIT_1;
    return BFSK_BIT_INVALID;
}

void bsw_bfsk_mod_set_freq(uint32_t freq)
{
    (void)freq;
    /* TODO: 通过 mcal_spi 配置 AD9833 寄存器 */
}

void bsw_bfsk_mod_bit(bsfk_bit_t bit)
{
    if (bit == BFSK_BIT_0) bsw_bfsk_mod_set_freq(g_cfg.f0);
    else                   bsw_bfsk_mod_set_freq(g_cfg.f1);
}

void bsw_bfsk_amp_enable(void)
{
    mcal_gpio_write(GPIO_PIN_AMP_EN, GPIO_HIGH);
}

void bsw_bfsk_amp_disable(void)
{
    mcal_gpio_write(GPIO_PIN_AMP_EN, GPIO_LOW);
}
