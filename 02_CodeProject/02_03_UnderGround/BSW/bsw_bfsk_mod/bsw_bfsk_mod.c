/**
 * @file    bsw_bfsk_mod.c
 * @brief   BFSK 调制器 - BSW 层实现
 *
 * 调制链路：
 *   1. 字节流 → bit 流（MSB first）
 *   2. bit 流 → 频率序列（bit=0 → f0，bit=1 → f1）
 *   3. 每个 bit 持续 bit_period_ms，通过定时器控制
 *   4. 调用 bsw_ad9833_select() 切换频率
 *
 * 状态机：
 *   IDLE → 调用 send() → SENDING
 *   SENDING → 逐 bit 发送，定时器驱动 → 所有 bit 完成 → DONE_OK
 *   DONE_OK → 应用层确认后，手动转 IDLE（或自动转）
 *
 * @dependency  bsw_ad9833  (频率切换)
 *              mcal_timer  (定时器，控制 bit 速率)
 */

#include "bsw_bfsk_mod.h"
#include "bsw_ad9833.h"

#include <string.h>

/* ========== 内部状态 ========== */

typedef struct {
    uint16_t f0_hz;                         /* 低频点（bit=0） */
    uint16_t f1_hz;                         /* 高频点（bit=1） */
    uint32_t bit_period_ms;                 /* 每 bit 持续时间 */
    
    uint8_t  tx_buf[BSW_BFSK_MOD_MAX_BYTES]; /* 待发送字节缓冲 */
    uint16_t tx_len;                        /* 字节数 */
    uint16_t tx_bit_count;                  /* 当前字节的总 bit 数 */
    
    uint16_t current_bit_idx;               /* 当前正在发送的 bit 索引 */
    uint32_t bit_start_ms;                  /* 当前 bit 开始时刻 */
    
    bsw_bfsk_mod_state_t state;             /* 状态机 */
} bsw_bfsk_mod_ctx_t;

static bsw_bfsk_mod_ctx_t s_ctx = {0};

/* ========== 内部函数 ========== */

/**
 * @brief   从字节缓冲中提取第 bit_idx 个 bit（MSB first）
 * @param   byte_buf   字节缓冲
 * @param   bit_idx    第几个 bit（从 0 开始，全局计数）
 * @retval  0 或 1
 */
static uint8_t _get_bit(const uint8_t *byte_buf, uint16_t bit_idx)
{
    uint16_t byte_idx = bit_idx / 8u;
    uint8_t bit_pos = 7u - (bit_idx % 8u);  /* MSB first */
    return (byte_buf[byte_idx] >> bit_pos) & 1u;
}

/**
 * @brief   发送当前 bit（切换 AD9833 频率）
 * @note    根据当前 bit 值切换到 f0 或 f1
 */
static void _send_current_bit(void)
{
    if (s_ctx.current_bit_idx >= s_ctx.tx_bit_count) {
        return;  /* 已发送完所有 bit */
    }
    
    uint8_t bit_val = _get_bit(s_ctx.tx_buf, s_ctx.current_bit_idx);
    bsw_ad9833_reg_t reg = (bit_val == 0) ? BSW_AD9833_REG_0 : BSW_AD9833_REG_1;
    
    bsw_ad9833_select(reg);  /* 切换频率 */
}

/* ========== 公共 API 实现 ========== */

bsw_bfsk_mod_ret_t bsw_bfsk_mod_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = BSW_BFSK_MOD_IDLE;
    return BSW_BFSK_MOD_OK;
}

bsw_bfsk_mod_ret_t bsw_bfsk_mod_set_freq(uint16_t f0_hz,
                                         uint16_t f1_hz,
                                         uint32_t bit_period_ms)
{
    if (f0_hz == 0 || f1_hz == 0 || bit_period_ms == 0) {
        return BSW_BFSK_MOD_ERR_PARAM;
    }
    
    s_ctx.f0_hz = f0_hz;
    s_ctx.f1_hz = f1_hz;
    s_ctx.bit_period_ms = bit_period_ms;
    
    /* 装载频率到 AD9833 */
    if (bsw_ad9833_set_freq(BSW_AD9833_REG_0, f0_hz) != BSW_AD9833_OK) {
        return BSW_BFSK_MOD_ERR_HW;
    }
    if (bsw_ad9833_set_freq(BSW_AD9833_REG_1, f1_hz) != BSW_AD9833_OK) {
        return BSW_BFSK_MOD_ERR_HW;
    }
    
    return BSW_BFSK_MOD_OK;
}

bsw_bfsk_mod_ret_t bsw_bfsk_mod_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > BSW_BFSK_MOD_MAX_BYTES) {
        return BSW_BFSK_MOD_ERR_PARAM;
    }
    
    if (s_ctx.state != BSW_BFSK_MOD_IDLE) {
        return BSW_BFSK_MOD_ERR_STATE;  /* 正在发送或出错中 */
    }
    
    /* 拷贝数据到内部缓冲 */
    memcpy(s_ctx.tx_buf, data, len);
    s_ctx.tx_len = len;
    s_ctx.tx_bit_count = len * 8u;
    
    /* 初始化发送状态 */
    s_ctx.current_bit_idx = 0u;
    s_ctx.bit_start_ms = 0u;  /* 等 run() 更新为当前时刻 */
    
    /* 状态切换为发送中 */
    s_ctx.state = BSW_BFSK_MOD_SENDING;
    
    return BSW_BFSK_MOD_OK;
}

void bsw_bfsk_mod_run(uint32_t now_tick)
{
    if (s_ctx.state != BSW_BFSK_MOD_SENDING) {
        return;
    }
    
    /* 第一个 bit：初始化起始时刻 */
    if (s_ctx.current_bit_idx == 0) {
        s_ctx.bit_start_ms = now_tick;
        _send_current_bit();  /* 立即发送第 0 个 bit 对应的频率 */
        return;
    }
    
    /* 检查是否到了切换时刻 */
    uint32_t elapsed_ms = (int32_t)(now_tick - s_ctx.bit_start_ms);
    uint32_t target_bit_idx = elapsed_ms / s_ctx.bit_period_ms;
    
    /* 未到切换时刻 */
    if (target_bit_idx == s_ctx.current_bit_idx) {
        return;
    }
    
    /* 切换到下一个 bit */
    s_ctx.current_bit_idx = target_bit_idx;
    
    /* 检查是否发送完所有 bit */
    if (s_ctx.current_bit_idx >= s_ctx.tx_bit_count) {
        s_ctx.state = BSW_BFSK_MOD_DONE_OK;
        return;
    }
    
    _send_current_bit();
}

bsw_bfsk_mod_state_t bsw_bfsk_mod_get_state(void)
{
    return s_ctx.state;
}

bsw_bfsk_mod_ret_t bsw_bfsk_mod_stop(void)
{
    s_ctx.state = BSW_BFSK_MOD_IDLE;
    s_ctx.current_bit_idx = 0u;
    s_ctx.tx_len = 0u;
    return BSW_BFSK_MOD_OK;
}

/* ========== 定时器回调 ========== */

void bsw_bfsk_mod_on_bit_timer(void)
{
    /* 该回调由 mcal_timer 在每个 bit_period_ms 时刻调用
     * 但在当前实现中，我们在 run() 中通过时间轮询实现
     * 如果需要硬定时器精确控制，可以在这里推进状态
     */
    
    if (s_ctx.state != BSW_BFSK_MOD_SENDING) {
        return;
    }
    
    /* 推进到下一个 bit */
    s_ctx.current_bit_idx++;
    
    if (s_ctx.current_bit_idx >= s_ctx.tx_bit_count) {
        s_ctx.state = BSW_BFSK_MOD_DONE_OK;
        return;
    }
    
    _send_current_bit();
}
