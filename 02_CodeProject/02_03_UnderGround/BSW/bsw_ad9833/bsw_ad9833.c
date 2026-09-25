/**
 * @file    bsw_ad9833.c
 * @brief   AD9833 DDS 驱动 - BSW 层实现
 *
 * @note    SPI 帧格式（数据手册 §Table 5）：
 *   - 每帧 16 bit，MSB 先发
 *   - D15:D14 = 寄存器选择：00=CTRL / 01=FREQ0 / 10=FREQ1 / 11=PHASE
 *   - FREQx 写 28 bit 需拆两次：先发 LSB 14 + B28=1，再发 MSB 14
 *
 * 时序关键点：
 *   - FSYNC 拉低即开始，MSB 在 SCLK 下降沿之前稳定
 *   - 本模块用软件控制 GPIO 实现 FSYNC（非 SPI NSS），
 *     所以 mcal_spi 必须配 SPI_NSS_SOFT（已配：mcal_spi.c）。
 *
 * SPI Mode 必须 = Mode 2（已配：CPOL=HIGH, CPHA=1EDGE）。
 */

#include "bsw_ad9833.h"
#include "mcal_gpio.h"
#include "mcal_spi.h"
#include "bsw_log.h"        /* 可选：调试日志 */

/* ========== AD9833 命令字常量（D15:D14 = 寄存器选择） ========== */
#define AD_REG_CTRL    0x0000   /* 控制寄存器 */
#define AD_REG_FREQ0   0x4000   /* FREQ0 起始地址（写入时按数据手册 28-bit 拆两次） */
#define AD_REG_FREQ1   0x8000   /* FREQ1 */
#define AD_REG_PHASE0  0xC000   /* PHASE0 */
#define AD_REG_PHASE1  0xE000   /* PHASE1 */

/* 控制寄存器位（D13..D0） */
#define CTRL_B28       0x2000   /* 完成 28 位频率字时设 */
#define CTRL_HLB       0x1000   /* 写 MSB 4 位（不常用） */
#define CTRL_FSELECT   0x0800   /* 0=FREQ0, 1=FREQ1 */
#define CTRL_PSELECT   0x0400   /* 0=PHASE0, 1=PHASE1（仅在 PHASE 寄存器内写入时用） */
#define CTRL_RESET     0x0100   /* 1=DDS 内部复位，MCLK 计数器清零 */
#define CTRL_SLEEP12   0x0080   /* 1=关 MCLK（EXT_CLK 缓冲），可用于省电 */
#define CTRL_SLEEP1    0x0040   /* 1=DAC 掉电，输出隔离 */
#define CTRL_OPBITEN   0x0020   /* 1=输出 MSB 端（方波模式） */
#define CTRL_DIV2      0x0008   /* 1=输出 MSB 时二分频 */
#define CTRL_MODE      0x0002   /* 1=三角波，0=正弦波 */

/* 方波组合 = OPBITEN | DIV2（让方波从 MSB 引脚拿，输出端实际反向处理） */
#define CTRL_SQUARE    (CTRL_OPBITEN | CTRL_DIV2)

/* ========== 内部状态 ========== */
static uint32_t s_mclk_hz        = 0;
static bsw_ad9833_wave_t s_wave  = BSW_AD9833_WAVE_SINE;
static bsw_ad9833_reg_t  s_fsel  = BSW_AD9833_REG_0;
static uint8_t           s_inited = 0;

/* ========== 内部：SPI 一帧 16 bit 写入 ========== */
/**
 * @brief   写入一个 16-bit 命令到 AD9833
 * @note    CS 控制由 FSYNC GPIO 完成，时序：
 *          FSYNC LOW → 2 字节 → FSYNC HIGH
 *          mcal_spi_transfer_buf 内部阻塞发送，时长远大于 AD9833 所需
 *          的 40 ns MCLK 复位时间，无需延时。
 */
static void _write_cmd(uint16_t word)
{
    uint8_t tx[2];
    tx[0] = (uint8_t)(word >> 8);
    tx[1] = (uint8_t)(word & 0xFF);

    mcal_gpio_write(GPIO_PIN_AD9833_FSYNC, GPIO_LOW);
    mcal_spi_transfer_buf(SPI_ID_1, tx, NULL, 2);
    mcal_gpio_write(GPIO_PIN_AD9833_FSYNC, GPIO_HIGH);
}

/* ========== 内部：组装当前 control 寄存器值（保持 B28=0、当前 FSELECT/波形/模式） ========== */
static uint16_t _build_ctrl(void)
{
    uint16_t ctrl = AD_REG_CTRL;

    if (s_fsel == BSW_AD9833_REG_1) {
        ctrl |= CTRL_FSELECT;
    }
    switch (s_wave) {
        case BSW_AD9833_WAVE_TRIANGLE: ctrl |= CTRL_MODE;        break;
        case BSW_AD9833_WAVE_SQUARE:   ctrl |= CTRL_SQUARE;      break;
        case BSW_AD9833_WAVE_SINE:
        default: break;
    }

    return ctrl;
}

/* ================================================================ */
/*                           公共接口实现                            */
/* ================================================================ */

bsw_ad9833_ret_t bsw_ad9833_init(uint32_t mclk_hz)
{
    if (mclk_hz == 0) {
        return BSW_AD9833_ERR_PARAM;
    }
    if (s_inited) {
        return BSW_AD9833_OK;     /* 幂等：已初始化直接返回成功 */
    }

    s_mclk_hz = mclk_hz;

    /* 1. FSYNC GPIO 配置（PC4 推挽输出，默认高电平） */
    mcal_gpio_set_output(GPIO_PIN_AD9833_FSYNC);
    mcal_gpio_write(GPIO_PIN_AD9833_FSYNC, GPIO_HIGH);

    /* 2. AD9833 复位序列
     *    数据手册 §INITIAL POWER-UP：写 0x0100（RESET=1）后需等约 1 个 MCLK 周期。
     *    SPI 一次 transfer 至少几 μs，远超 40 ns，故无需额外 delay。 */
    _write_cmd(AD_REG_CTRL | CTRL_RESET);

    /* 3. 清零 FREQ0 / FREQ1（拆两次：LSB 14 + B28=1，后跟 MSB 14） */
    _write_cmd(AD_REG_FREQ0 | CTRL_B28);     /* FREQ0 LSB 14 = 0, B28=1 */
    _write_cmd(AD_REG_FREQ0);                /* FREQ0 MSB 14 = 0 */
    _write_cmd(AD_REG_FREQ1 | CTRL_B28);     /* FREQ1 LSB 14 = 0, B28=1 */
    _write_cmd(AD_REG_FREQ1);                /* FREQ1 MSB 14 = 0 */

    /* 4. 清零 PHASE0 / PHASE1（共 12 位相位偏移） */
    _write_cmd(AD_REG_PHASE0);               /* PHASE0 = 0 */
    _write_cmd(AD_REG_PHASE1);               /* PHASE1 = 0 */

    /* 5. 退出复位（默认正弦波、FREQ0） */
    s_fsel = BSW_AD9833_REG_0;
    s_wave = BSW_AD9833_WAVE_SINE;
    _write_cmd(_build_ctrl());

    s_inited = 1;

    bsw_log("[AD9833] init OK, MCLK=%lu Hz, FSYNC=PC4, Mode=2\r\n", (unsigned long)mclk_hz);
    return BSW_AD9833_OK;
}

bsw_ad9833_ret_t bsw_ad9833_set_freq(bsw_ad9833_reg_t reg, uint32_t freq_hz)
{
    if (!s_inited || (reg != BSW_AD9833_REG_0 && reg != BSW_AD9833_REG_1)) {
        return BSW_AD9833_ERR_PARAM;
    }
    /* 频率上限检查（Nyquist ≥ MCLK/2 仅在正弦时理论可达，工程保留 < MCLK/4 更安全） */
    if (freq_hz == 0 || freq_hz > s_mclk_hz / 4) {
        bsw_log("[AD9833] freq out of range: %lu (MCLK/4=%lu)\r\n",
                (unsigned long)freq_hz, (unsigned long)(s_mclk_hz / 4));
        return BSW_AD9833_ERR_PARAM;
    }

    /* freq_word = freq_hz × 2^28 / MCLK */
    uint64_t num = (uint64_t)freq_hz * (1ULL << 28);
    uint32_t freq_word = (uint32_t)(num / s_mclk_hz);

    uint16_t reg_base = (reg == BSW_AD9833_REG_1) ? AD_REG_FREQ1 : AD_REG_FREQ0;

    /* 第一次写：LSB 14 + B28=1 */
    uint16_t lsb = reg_base | CTRL_B28 | (uint16_t)(freq_word & 0x3FFF);
    /* 第二次写：MSB 14 */
    uint16_t msb = reg_base | (uint16_t)((freq_word >> 14) & 0x3FFF);

    _write_cmd(lsb);
    _write_cmd(msb);

    return BSW_AD9833_OK;
}

bsw_ad9833_ret_t bsw_ad9833_select(bsw_ad9833_reg_t reg)
{
    if (!s_inited || (reg != BSW_AD9833_REG_0 && reg != BSW_AD9833_REG_1)) {
        return BSW_AD9833_ERR_PARAM;
    }
    s_fsel = reg;
    _write_cmd(_build_ctrl());
    return BSW_AD9833_OK;
}

bsw_ad9833_ret_t bsw_ad9833_set_waveform(bsw_ad9833_wave_t wave)
{
    if (!s_inited) {
        return BSW_AD9833_ERR_PARAM;
    }
    s_wave = wave;
    _write_cmd(_build_ctrl());
    return BSW_AD9833_OK;
}

bsw_ad9833_ret_t bsw_ad9833_sleep(uint8_t sleep)
{
    if (!s_inited) {
        return BSW_AD9833_ERR_PARAM;
    }
    uint16_t ctrl = _build_ctrl();

    if (sleep) {
        ctrl |= CTRL_SLEEP1;     /* DAC 掉电；保留 MCLK 便于快速唤醒 */
    }
    _write_cmd(ctrl);
    return BSW_AD9833_OK;
}

bsw_ad9833_ret_t bsw_ad9833_reset(uint8_t reset)
{
    if (!s_inited) {
        return BSW_AD9833_ERR_PARAM;
    }
    /* 复位操作不影响内部频率/相位寄存器（手册 §Reset）
     * 只复位内部 NCO。退出复位前先把 waveform/select 设回去。 */
    uint16_t ctrl = _build_ctrl();
    if (reset) ctrl |= CTRL_RESET;
    _write_cmd(ctrl);
    return BSW_AD9833_OK;
}

uint8_t bsw_ad9833_is_ready(void)
{
    if (!s_inited) return 0;
    /* 简易 ping：拉低 RESET→读回状态位→释放
     * AD9833 无读取路径，所以判断方式为"FSYNC、SPI 句柄、配置有效"。
     * 实际工程中此处可加"读 SPI 错误计数"或"超时看门狗自检"。 */
    return (s_mclk_hz > 0) ? 1 : 0;
}
