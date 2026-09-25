/**
 * @file    bsw_ad9833.h
 * @brief   AD9833 DDS 驱动 - BSW 层
 *
 * @note    AD9833 是 Analog Devices 的可编程波形发生器（DDS）。
 *          本驱动通过 mcal_spi + 1 个软件控制 GPIO（FSYNC）控制芯片。
 *
 * 关键寄存器：
 *   - 0x0000: 控制寄存器（RESET, SLEEP, FSELECT, PSEL, 波形模式…）
 *   - 0x4000: FREQ0（频率字 28 bit，分两次写）
 *   - 0x8000: FREQ1（同上）
 *   - 0xC000: PHASE0（相位偏移 12 bit）
 *   - 0xE000: PHASE1（同上）
 *
 * 频率分辨率：
 *   freq_word  = freq_hz × 2^28 / MCLK_Hz
 *   实际输出   = freq_word × MCLK_Hz / 2^28
 *
 * @dependency  mcal_spi     (SPI 字节流)
 *              mcal_gpio    (FSYNC: GPIO_PIN_AD9833_FSYNC，默认 PC4)
 *
 * @usage
 *   bsw_ad9833_init(25000000U);                  // MCLK = 25 MHz
 *   bsw_ad9833_set_freq(BSW_AD9833_REG_0, 5000); // 5 kHz
 *   bsw_ad9833_set_freq(BSW_AD9833_REG_1, 10000);// 10 kHz
 *   bsw_ad9833_select(BSW_AD9833_REG_0);          // BFSK 调制：切频率
 */

#ifndef BSW_AD9833_H
#define BSW_AD9833_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 频率/相位寄存器选择 ========== */
typedef enum {
    BSW_AD9833_REG_0 = 0,   /* FREQ0 / PHASE0 */
    BSW_AD9833_REG_1 = 1,   /* FREQ1 / PHASE1 */
} bsw_ad9833_reg_t;

/* ========== 输出波形选择 ========== */
typedef enum {
    BSW_AD9833_WAVE_SINE     = 0,   /* 正弦波（默认，初始化后即此模式） */
    BSW_AD9833_WAVE_TRIANGLE = 1,   /* 三角波 */
    BSW_AD9833_WAVE_SQUARE   = 2,   /* 方波（OPBITEN|DIV2|MSB） */
} bsw_ad9833_wave_t;

/* ========== 返回码 ========== */
typedef enum {
    BSW_AD9833_OK        =  0,
    BSW_AD9833_ERR_PARAM = -1,   /* 参数越界 / 未初始化 */
    BSW_AD9833_ERR_SPI   = -2,   /* SPI 通信异常（保留，HAL 错误走 Error_Handler） */
} bsw_ad9833_ret_t;

/* ========== 函数声明 ========== */

/**
 * @brief   初始化 AD9833
 * @param   mclk_hz  AD9833 外部 MCLK 时钟频率（Hz），如 25_000_000
 * @retval  BSW_AD9833_OK
 *          BSW_AD9833_ERR_PARAM  mclk_hz 为 0
 *
 * @note    调用本函数前必须先：
 *          - mcal_spi_init(SPI_ID_1)  （SPI1, Mode 2）
 *          - mcal_gpio_init()         （PC4 默认未配置，本模块内会设输出）
 *
 *          流程：
 *          1. 配置 FSYNC 引脚为推挽输出，初始高电平
 *          2. RESET=1 进入复位状态
 *          3. 清零 FREQ0 / FREQ1 / PHASE0 / PHASE1
 *          4. 退出复位，默认正弦波
 */
bsw_ad9833_ret_t bsw_ad9833_init(uint32_t mclk_hz);

/**
 * @brief   装载某个频率寄存器的频率字
 * @param   reg     BSW_AD9833_REG_0 / REG_1
 * @param   freq_hz 目标频率（Hz），0 ~ MCLK_Hz/2
 * @retval  BSW_AD9833_OK
 *          BSW_AD9833_ERR_PARAM
 */
bsw_ad9833_ret_t bsw_ad9833_set_freq(bsw_ad9833_reg_t reg, uint32_t freq_hz);

/**
 * @brief   切换当前输出使用的频率寄存器（BFSK 调制核心调用）
 * @param   reg  BSW_AD9833_REG_0 / REG_1
 * @retval  BSW_AD9833_OK / ERR_PARAM
 * @note    设置 FREQ0 / FREQ1 各自的频率后，本函数切换 FSELECT 位
 *          实现无相位差突变的频率跳变。
 */
bsw_ad9833_ret_t bsw_ad9833_select(bsw_ad9833_reg_t reg);

/**
 * @brief   设置输出波形类型
 * @param   wave  SINE / TRIANGLE / SQUARE
 * @retval  BSW_AD9833_OK / ERR_PARAM
 */
bsw_ad9833_ret_t bsw_ad9833_set_waveform(bsw_ad9833_wave_t wave);

/**
 * @brief   进入 / 退出掉电模式
 * @param   sleep  1=DAC 与内部时钟全掉电（高速低功耗）；0=正常工作
 * @retval  BSW_AD9833_OK / ERR_PARAM
 */
bsw_ad9833_ret_t bsw_ad9833_sleep(uint8_t sleep);

/**
 * @brief   复位 / 解除复位
 * @param   reset  1=进入复位（内部寄存器恢复默认值，但 DDS 立即归零）
 *                   ；0=解除复位，输出恢复
 * @retval  BSW_AD9833_OK / ERR_PARAM
 */
bsw_ad9833_ret_t bsw_ad9833_reset(uint8_t reset);

/**
 * @brief   自检：探一次复位位读写，判断 AD9833 是否响应
 * @retval  1=响应正常  0=异常 / 未初始化
 */
uint8_t bsw_ad9833_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_AD9833_H */
