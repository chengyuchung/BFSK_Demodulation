/**
 * @file    bsw_bfsk_demod.c
 * @brief   BFSK 解调器 - BSW 层实现（占位）
 *
 * 当前状态：空壳，待填充。
 * 实现路线图：
 *   1. 注册 TIM1 IC 回调
 *   2. 边沿 Δt → 频率判定（容忍窗口 ±20% 之类）
 *   3. bit 滑窗积累
 *   4. 帧头检测 + 数据提取 + CRC 校验
 *   5. 超时管理 + 状态机推进
 *   6. 通知 app_node_fsm（on_ack_received 或新回调）
 *
 * @dependency  bsw_bfsk_demod.h
 *              mcal_timer.h        (TIM1 IC 回调注册)
 *              app_node_fsm.h      (注入 cur_freq，结果回传)
 */

#include "bsw_bfsk_demod.h"

#include <stddef.h>   /* NULL 定义 */

/* TODO: 加实际需要的 include (HAL / mcal_timer / app_node_fsm / app_node_ctx) */

/* ========== 静态上下文（占位） ========== */
/* TODO: 替换为实际状态字段 */
static bsw_bfsk_demod_state_t s_state = BSW_BFSK_DEMOD_IDLE;

/* ========== 公共 API 实现（占位） ========== */

bsw_bfsk_demod_ret_t bsw_bfsk_demod_init(void)
{
    s_state = BSW_BFSK_DEMOD_IDLE;
    /* TODO: mcal_timer_ic_register_callback(bsw_bfsk_demod_on_ic_edge); */
    return BSW_BFSK_DEMOD_OK;
}

bsw_bfsk_demod_ret_t bsw_bfsk_demod_start(uint16_t f0_hz,
                                         uint16_t f1_hz,
                                         uint32_t timeout_ms)
{
    (void)f0_hz;
    (void)f1_hz;
    (void)timeout_ms;
    /* TODO:
     *   1. 参数合法性检查（f0_hz != 0, f1_hz != 0, |f1-f0| >= MIN_GAP_HZ）
     *   2. mcal_timer_ic_start(MCAL_TIMER_TIM1)
     *   3. 清空 s_bit_buf, s_bit_count
     *   4. s_timeout_tick = now + timeout_ms
     *   5. s_state = BSW_BFSK_DEMOD_RUNNING
     */
    s_state = BSW_BFSK_DEMOD_RUNNING;
    return BSW_BFSK_DEMOD_OK;
}

bsw_bfsk_demod_ret_t bsw_bfsk_demod_stop(void)
{
    /* TODO: mcal_timer_ic_stop(MCAL_TIMER_TIM1); */
    s_state = BSW_BFSK_DEMOD_IDLE;
    return BSW_BFSK_DEMOD_OK;
}

void bsw_bfsk_demod_run(uint32_t now_tick)
{
    (void)now_tick;
    /* TODO:
     *   1. 检查 s_state == RUNNING && now_tick >= s_timeout_tick
     *   2. 超时 → s_state = DONE_TIMEOUT, 关 IC
     */
}

bsw_bfsk_demod_state_t bsw_bfsk_demod_get_state(void)
{
    return s_state;
}

bsw_bfsk_demod_ret_t bsw_bfsk_demod_take_result(bsw_bfsk_demod_result_t *out)
{
    if (out == NULL) {
        return BSW_BFSK_DEMOD_ERR_PARAM;
    }
    /* TODO: 仅当 s_state == DONE_OK 时填 out；消费后回到 IDLE 或下一轮监听 */
    (void)out;
    return BSW_BFSK_DEMOD_ERR_STATE;
}

/* ========== 内部 IC 回调（占位） ========== */

void bsw_bfsk_demod_on_ic_edge(uint32_t tick_us)
{
    (void)tick_us;
    /* TODO:
     *   1. Δt = tick_us - s_last_edge_tick
     *   2. f_est = 1_000_000 / Δt
     *   3. 靠近 f0 → push 0；靠近 f1 → push 1；其他 → 复位
     *   4. bit 累积满 N 位后做帧解析
     */
}
