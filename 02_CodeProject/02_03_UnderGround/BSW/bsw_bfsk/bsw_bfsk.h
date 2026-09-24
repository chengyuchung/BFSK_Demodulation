/**
 * @file    bsw_bfsk.h
 * @brief   BFSK 调制解调 - BSW 层
 * @note    包含 Goertzel 算法 + 半周期校验 + 判决输出
 *
 * @dependency  mcal_adc (采样数据)
 * @dependency  mcal_spi  (AD9833 调制)
 */

#ifndef BSW_BFSK_H
#define BSW_BFSK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 宏定义 ========== */
#define BFSK_F0_DEFAULT   200000U   /* 默认频率 f0 = 200kHz */
#define BFSK_F1_DEFAULT   250000U   /* 默认频率 f1 = 250kHz */
#define BFSK_SAMPLE_RATE  2000000U  /* ADC 采样率 2MHz */
#define BFSK_POINTS_MIN   50         /* 单频率最小采样点数 */

/* ========== 类型定义 ========== */

/* 解调结果 */
typedef enum {
    BFSK_BIT_0 = 0,
    BFSK_BIT_1 = 1,
    BFSK_BIT_INVALID = -1
} bsfk_bit_t;

/* BFSK 模块配置 */
typedef struct {
    uint32_t f0;       /* 频率 0 (Hz) */
    uint32_t f1;       /* 频率 1 (Hz) */
    uint32_t sample_rate;
    uint16_t threshold; /* Goertzel 幅值判决阈值 */
} bsw_bfsk_config_t;

/* ========== 函数声明 ========== */

/* ---- 解调侧 ---- */

/**
 * @brief   初始化 BFSK 解调器
 * @param   cfg   配置参数
 */
void bsw_bfsk_init(const bsw_bfsk_config_t *cfg);

/**
 * @brief   从一段采样数据中解调出一个比特
 * @param   samples  ADC 采样数据
 * @param   len      数据长度
 * @retval  BFSK_BIT_0 / BFSK_BIT_1 / BFSK_BIT_INVALID
 */
bsfk_bit_t bsw_bfsk_demod_bit(const int16_t *samples, uint32_t len);

/**
 * @brief   Goertzel 算法计算指定频率的幅值
 * @param   samples  采样数据
 * @param   len     数据长度
 * @param   freq    要检测的频率 (Hz)
 * @param   fs      采样率 (Hz)
 * @retval  幅值（归一化整数）
 */
int32_t bsw_goertzel(const int16_t *samples, uint32_t len, uint32_t freq, uint32_t fs);

/* ---- 调制侧 ---- */

/**
 * @brief   配置 AD9833 输出指定频率
 * @param   freq   频率 (Hz)
 */
void bsw_bfsk_mod_set_freq(uint32_t freq);

/**
 * @brief   发送一个比特（驱动 AD9833 切换频率）
 * @param   bit    BFSK_BIT_0 / BFSK_BIT_1
 * @note    实际切换由 mcal_spi 配合 AD9833 完成
 */
void bsw_bfsk_mod_bit(bsfk_bit_t bit);

/**
 * @brief   功放使能
 */
void bsw_bfsk_amp_enable(void);

/**
 * @brief   功放关闭
 */
void bsw_bfsk_amp_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_BFSK_H */
