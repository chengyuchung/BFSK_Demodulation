/**
 * @file    app_node_fsm.c
 * @brief   井下节点状态机实现
 *
 * 状态转移遵循 扫频启动协议.md：
 *   - §一  扫频启动识别（三重物理特征校验）
 *   - §二  扫频结束判定（850 ms 硬超时 + 尾部静默检测）
 *   - §五  断链角色仲裁（4 标志位事务上下文）
 *   - §六  全生命周期休眠逻辑（清洁态鉴别）
 */

#include "app_node_fsm.h"

#include <string.h>             /* memset */

#include "main.h"                /* HAL_GetTick, Error_Handler */
#include "bsw_adc_ringbuf.h"
#include "bsw_ad9833.h"
#include "bsw_node_id.h"            /* 节点地址模块：本机 hard_id 注入 */
#include "mcal_timer.h"          /* mcal_timer_ic_start/stop，FSM 直接控制 TIM1 输入捕获 */

/* ========== 静态全局上下文 ========== */
/* C 标准保证 static 存储期对象零初始化，无需 = {0}，避免与枚举混用的告警 */
static app_node_ctx_t s_ctx;

/* ========== 私有辅助 ========== */

/**
 * @brief   频点 → amp_table 索引的"就近映射"（协议 §三.2）
 * @param   f_hz  扫频检测算法解出的瞬时频率
 * @return  0~18 合法索引；0xFF 表示落在空白带 / 工频带 / 容差外，应丢弃
 * @note    19 个标称频点：125, 175, 225, ..., 1025 Hz（步进 50 Hz）。
 *          容差 ±15 Hz：超过这个窗口的频点视为无效，不写入 amp_table。
 */
static uint8_t _sweep_freq_to_index(uint16_t f_hz)
{
    if (f_hz < (uint16_t)(SWEEP_FREQ_START - SWEEP_FREQ_TOLERANCE_HZ)) {
        return 0xFF;
    }

    int32_t delta = (int32_t)f_hz - (int32_t)SWEEP_FREQ_START;
    if (delta < 0) {
        delta = 0;
    }

    int32_t idx     = delta / (int32_t)SWEEP_FREQ_STEP;
    int32_t offset  = delta % (int32_t)SWEEP_FREQ_STEP;

    if (offset > (int32_t)(SWEEP_FREQ_STEP / 2)) {
        ++idx;
        offset = (int32_t)SWEEP_FREQ_STEP - offset;
    }

    if (idx < 0 || idx >= (int32_t)SWEEP_FREQ_COUNT) {
        return 0xFF;
    }

    if (offset > (int32_t)SWEEP_FREQ_TOLERANCE_HZ) {
        return 0xFF;
    }

    return (uint8_t)idx;
}

/**
 * @brief   协议 §三.3 "只记录，不淘汰" 写 amp_table
 * @note    同一频点多次命中时，只保留幅值最大的那次，绝不丢弃前面的记录。
 *          这是协议抗频点衰减的核心机制：哪怕某个频点信号中途掉到 0，
 *          历史最大幅值仍会留在表里给频对决选用。
 */
static void _update_amp_table(uint8_t idx, uint16_t amplitude)
{
    if (idx >= SWEEP_FREQ_COUNT) {
        return;
    }
    if (amplitude > s_ctx.sweep_amp_table[idx]) {
        s_ctx.sweep_amp_table[idx] = amplitude;
    }
}

/**
 * @brief   频对决选算法（协议 §三.4）
 * @param[out] out_f0  选出的低频点
 * @param[out] out_f1  选出的高频点
 * @retval   DECISION_OK           选出有效 (f0, f1)
 * @retval   DECISION_ERR_NO_VALID amp_table 里 amp > noise_floor + delta 的格 < 2
 * @retval   DECISION_ERR_NO_PAIR  找不出和 f0_idx 间距 ≥ SWEEP_MIN_FREQ_GAP_HZ 的次大候选
 * @note    算法 3 步：
 *          1) 筛有效频点：amp_table[i] > noise_floor + SWEEP_VALID_AMP_DELTA
 *          2) 找最强作为 f0_idx
 *          3) 在剩下的有效候选里，找 amp 次大 + 间距 ≥ SWEEP_MIN_FREQ_GAP_HZ 的作为 f1_idx
 *          协议 §三 要求|f1-f0| ≥ 150 Hz（防止 BFSK 解调频率过近失败）。
 */
static int32_t _select_freq_pair(uint16_t *out_f0, uint16_t *out_f1)
{
    uint8_t valid_idx[SWEEP_FREQ_COUNT];
    uint8_t valid_cnt = 0;
    for (uint8_t i = 0; i < SWEEP_FREQ_COUNT; ++i) {
        uint16_t amp = s_ctx.sweep_amp_table[i];
        if (amp > (uint16_t)(s_ctx.noise_floor + SWEEP_VALID_AMP_DELTA)) {
            valid_idx[valid_cnt++] = i;
        }
    }
    if (valid_cnt < 2) {
        return -1;
    }

    uint8_t f0_idx = valid_idx[0];
    for (uint8_t k = 1; k < valid_cnt; ++k) {
        if (s_ctx.sweep_amp_table[valid_idx[k]] > s_ctx.sweep_amp_table[f0_idx]) {
            f0_idx = valid_idx[k];
        }
    }
    /* best_f0 先填好，ERR_NO_PAIR 时调用方也能拿到最强频点 */
    *out_f0 = (uint16_t)SWEEP_FREQ_START + (uint16_t)f0_idx * SWEEP_FREQ_STEP;

    uint8_t  f1_idx       = 0xFF;
    uint16_t f1_amp_best  = 0;
    for (uint8_t k = 0; k < valid_cnt; ++k) {
        uint8_t cand = valid_idx[k];
        if (cand == f0_idx) {
            continue;
        }
        uint8_t  gap_idx = (cand > f0_idx) ? (cand - f0_idx) : (f0_idx - cand);
        uint16_t gap_hz  = (uint16_t)gap_idx * SWEEP_FREQ_STEP;
        if (gap_hz < SWEEP_MIN_FREQ_GAP_HZ) {
            continue;
        }
        if (s_ctx.sweep_amp_table[cand] > f1_amp_best) {
            f1_amp_best = s_ctx.sweep_amp_table[cand];
            f1_idx       = cand;
        }
    }
    if (f1_idx == 0xFF) {
        return -2;
    }

    *out_f1 = (uint16_t)SWEEP_FREQ_START + (uint16_t)f1_idx * SWEEP_FREQ_STEP;
    return 0;
}

/**
 * @brief   内部状态转移（仅本文件调用）
 * @note    每次转移同步记录 state_enter_tick
 */
static void _state_transition(node_state_t next_state, uint32_t now_tick)
{
    if (s_ctx.state == next_state) {
        return;
    }

    s_ctx.state = next_state;
    s_ctx.state_enter_tick = now_tick;

    /* ---- 状态进入动作（Entry Actions） ----
     *
     * ADC / TIM6（采样） vs TIM1 IC（解调）的状态归属：
     *
     *   | 状态          | ADC ringbuf | TIM6 | TIM1 IC | 用途                  |
     *   |---------------|-------------|------|---------|-----------------------|
     *   | BOOT          | disable     | stop | stop    | 时钟/外设初始化       |
     *   | SCAN_LISTEN   | enable      | run  | stop    | 扫频全过程：填 19 格 amp_table |
     *   | SCAN          | disable     | stop | stop    | 我们主动发频对通知     |
     *   | LINKED        | disable     | stop | start   | BFSK 帧收发           |
     *   | SLEEP         | disable     | stop | stop    | 间歇休眠              |
     *   | FAULT         | disable     | stop | stop    | 故障态静默            |
     *
     *   重要（仅 SCAN_LISTEN 阶段填表）：
     *     - SCAN_LISTEN 才是 ADC 采样状态，amp_table 19 格在此期间填满；
     *     - on_sweep_detected() 在 SCAN_LISTEN 内被多次调用，每次写一格；
     *     - 850 ms 倒计时从首次命中起算，到期后由 run() 自动结算。
     */
    switch (next_state) {
        case NODE_BOOT:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            break;

        case NODE_SCAN_LISTEN:
            bsw_adc_ringbuf_enable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            memset(s_ctx.sweep_amp_table, 0, sizeof(s_ctx.sweep_amp_table));
            s_ctx.sweep_start_tick   = 0;
            s_ctx.sweep_timeout_tick = 0;
            break;

        case NODE_SCAN:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            /* 850 ms 倒计时接力继承自 SCAN_LISTEN 阶段首次命中时刻，
             * 不重新设 now_tick，避免双重倒计时 */
            if (s_ctx.sweep_start_tick == 0) {
                s_ctx.sweep_start_tick   = now_tick;
                s_ctx.sweep_timeout_tick = now_tick + 850U;
            }
            break;

        case NODE_LINKED:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_start(MCAL_TIMER_TIM1);
            break;

        case NODE_SLEEP:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            break;

        case NODE_PRE_LINKED:
            /* 启动三段子状态机，初始 phase = QUIET_OBSERVE（协议 §4.2.1）
             * PHASE1 实际等待时长 = bsw_node_id_get_reply_delay_ms() = hard_id × 200 ms
             *     中继 1：200 ms    中继 2：400 ms    井下：3000 ms
             * 注：PHASE1 内部用 40 ms 节拍器调 bsw_adc_ringbuf_calc_rms_sq() 检测应答 */
            s_ctx.pre_linked_last_check_tick = now_tick;
            s_ctx.pre_linked_phase           = PRE_QUIET_OBSERVE;
            s_ctx.pre_phase_enter_tick       = now_tick;
            s_ctx.self_reply_cur_is_f0       = 1U;
            break;

        case NODE_FAULT:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            break;

        default:
            break;
    }
}

/**
 * @brief   判断总线是否静默超时（清洁态鉴别）
 * @note    协议 §6.2：总线静默后先等 3~5 s，再降级休眠
 */
static bool _is_bus_quiet(uint32_t now_tick)
{
    return (s_ctx.quiet_start_tick > 0)
           && ((now_tick - s_ctx.quiet_start_tick) >= 3000U);
}

/* PRE_LINKED 三段子状态机（表驱动）*/
typedef void (*phase_handler_t)(uint32_t now_tick);

static void _phase_quiet_observe(uint32_t now_tick);
static void _phase_self_reply(uint32_t now_tick);
static void _phase_wait_ack(uint32_t now_tick);

static const phase_handler_t s_phase_handlers[] = {
    [PRE_QUIET_OBSERVE] = _phase_quiet_observe,
    [PRE_SELF_REPLY]    = _phase_self_reply,
    [PRE_WAIT_ACK]      = _phase_wait_ack,
};

/* PHASE1：等自己槽点（my_id × 200ms），期间 40ms 节拍器测总线避让他人回发 */
static void _phase_quiet_observe(uint32_t now_tick)
{
    if ((now_tick - s_ctx.pre_linked_last_check_tick) >= 40U) {
        s_ctx.pre_linked_last_check_tick = now_tick;
        uint32_t rms_sq = bsw_adc_ringbuf_calc_rms_sq();
        if (rms_sq > BSW_ADC_RINGBUF_SIGNAL_THRESHOLD) {
            s_ctx.cur_freq.f0_hz     = 0;
            s_ctx.cur_freq.f1_hz     = 0;
            s_ctx.sweep_result.f0_hz = 0;
            s_ctx.sweep_result.f1_hz = 0;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
            return;
        }
    }

    uint32_t my_slot_offset = bsw_node_id_get_reply_delay_ms();
    if (my_slot_offset == 0xFFFFFFFFu) {
        my_slot_offset = PRE_QUIET_OBSERVE_MAX_MS;
    }

    uint32_t since_state_enter = (uint32_t)(now_tick - s_ctx.state_enter_tick);
    if (since_state_enter < my_slot_offset
        && since_state_enter < PRE_QUIET_OBSERVE_MAX_MS) {
        return;
    }

    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, s_ctx.cur_freq.f0_hz);
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_1, s_ctx.cur_freq.f1_hz);
    (void)bsw_ad9833_select(BSW_AD9833_REG_0);

    s_ctx.pre_linked_phase         = PRE_SELF_REPLY;
    s_ctx.pre_phase_enter_tick     = now_tick;
    s_ctx.self_reply_cur_is_f0     = 1U;
    s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
}

/* PHASE2：在 (f0, f1) 上交替 BFSK 调制持续 2s */
static void _phase_self_reply(uint32_t now_tick)
{
    if ((now_tick - s_ctx.pre_phase_enter_tick) >= PRE_REPLY_DURATION_MS) {
        (void)bsw_ad9833_sleep(1U);
        s_ctx.pre_linked_phase     = PRE_WAIT_ACK;
        s_ctx.pre_phase_enter_tick = now_tick;
        return;
    }
    if ((int32_t)(now_tick - s_ctx.self_reply_next_bit_tick) < 0) {
        return;
    }
    if (s_ctx.self_reply_cur_is_f0) {
        (void)bsw_ad9833_select(BSW_AD9833_REG_1);
        s_ctx.self_reply_cur_is_f0 = 0U;
    } else {
        (void)bsw_ad9833_select(BSW_AD9833_REG_0);
        s_ctx.self_reply_cur_is_f0 = 1U;
    }
    s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
}

/* PHASE3：等井上 ACK（解调 API 未就绪，暂留空）*/
static void _phase_wait_ack(uint32_t now_tick)
{
    (void)now_tick;
    /* TODO: on_ack_received() 接入后切到 NODE_LINKED；超时回退后续补 */
}

static void _pre_linked_run(uint32_t now_tick)
{
    phase_handler_t h = s_phase_handlers[s_ctx.pre_linked_phase];
    if (h != NULL) {
        h(now_tick);
    }
}

/* ========== 公共 API 实现 ========== */

void app_node_fsm_init(uint8_t hard_id)
{
    memset(&s_ctx, 0, sizeof(s_ctx));

    s_ctx.hard_id = hard_id;
    s_ctx.noise_floor = SWEEP_DEFAULT_NOISE_FLOOR;

    bsw_node_id_init(hard_id);

    s_ctx.state_enter_tick = HAL_GetTick();
    _state_transition(NODE_SCAN_LISTEN, s_ctx.state_enter_tick);
}

void app_node_fsm_run(uint32_t now_tick)
{
    switch (s_ctx.state) {

        case NODE_BOOT:
            break;

        /* -------------------------------------------------------- */
        case NODE_SCAN_LISTEN:
            /* 850 ms 会话硬超时检查（协议 §二 机制 A）：
             *   - 倒计时从首次 on_sweep_detected 调用起算；
             *   - 到期后强制结算（哪怕 amp_table 没填满）；
             *   - 由 run() 在每 10 ms 主循环钩中检查触发。 */
            if (s_ctx.sweep_start_tick != 0
                && (now_tick - s_ctx.sweep_start_tick) >= 850U) {
                uint8_t cnt = 0;
                for (int i = 0; i < SWEEP_FREQ_COUNT; ++i) {
                    if (s_ctx.sweep_amp_table[i] > 0) {
                        ++cnt;
                    }
                }
                s_ctx.sweep_start_tick   = 0;
                s_ctx.sweep_timeout_tick = 0;
                app_node_fsm_on_sweep_complete(cnt);
            }
            break;

        /* -------------------------------------------------------- */
        case NODE_SCAN: {
            if ((now_tick - s_ctx.sweep_start_tick) >= 850U) {
                uint8_t cnt = 0;
                for (int i = 0; i < SWEEP_FREQ_COUNT; ++i) {
                    if (s_ctx.sweep_amp_table[i] > 0) {
                        ++cnt;
                    }
                }
                app_node_fsm_on_sweep_complete(cnt);
            }
            break;
        }

        /* -------------------------------------------------------- */
        case NODE_PRE_LINKED: {
            _pre_linked_run(now_tick);
            break;
        }

        /* -------------------------------------------------------- */
        case NODE_LINKED: {
            /* 工作态：4 标志位驱动断链检测。
             * 若链路断开（总线静默超时），退回 SCAN_LISTEN 重新扫频。 */
            if (s_ctx.flags.flag_fwd_up_ok == 0
                && s_ctx.flags.flag_fwd_dn_ok == 0) {
                if (_is_bus_quiet(now_tick)) {
                    _state_transition(NODE_SCAN_LISTEN, now_tick);
                }
            }
            break;
        }

        /* -------------------------------------------------------- */
        case NODE_SLEEP: {
            if (!app_node_fsm_is_clean()) {
                _state_transition(NODE_SCAN_LISTEN, now_tick);
            }
            break;
        }

        /* -------------------------------------------------------- */
        case NODE_FAULT:
            break;
    }
}

/* ---- 查询 API ---- */

node_state_t app_node_fsm_get_state(void)
{
    return s_ctx.state;
}

bool app_node_fsm_is_clean(void)
{
    return (s_ctx.flags.flag_rx_up_ok  == 0)
           && (s_ctx.flags.flag_fwd_dn_ok == 0)
           && (s_ctx.flags.flag_rx_dn_ok  == 0)
           && (s_ctx.flags.flag_fwd_up_ok == 0);
}

bool app_node_fsm_is_freq_locked(void)
{
    return (s_ctx.cur_freq.f0_hz != 0);
}

const app_node_ctx_t *app_node_fsm_get_ctx(void)
{
    return &s_ctx;
}

/* ========== 事件注入实现 ========== */

void app_node_fsm_on_sweep_detected(uint16_t f_hz, uint16_t amplitude, uint32_t now_tick)
{
    if (s_ctx.state != NODE_SCAN_LISTEN) {
        return;
    }

    uint8_t idx = _sweep_freq_to_index(f_hz);
    if (idx == 0xFF) {
        return;
    }

    _update_amp_table(idx, amplitude);

    /* 首次命中 → 启动 850 ms 会话倒计时（协议 §二 机制 A）*/
    if (s_ctx.sweep_start_tick == 0) {
        s_ctx.sweep_start_tick   = now_tick;
        s_ctx.sweep_timeout_tick = now_tick + 850U;
    }

    s_ctx.quiet_start_tick = 0;
}

void app_node_fsm_on_sweep_complete(uint8_t valid_freq_count)
{
    /* valid_freq_count 供粗筛诊断；严格判定（噪声底 + delta）由 _select_freq_pair 完成 */

    uint16_t best_f0 = 0, best_f1 = 0;
    int32_t  dr      = _select_freq_pair(&best_f0, &best_f1);
    if (dr == -1) {
        _state_transition(NODE_SCAN_LISTEN, HAL_GetTick());
        return;
    }
    if (dr == -2) {
        _state_transition(NODE_SCAN_LISTEN, HAL_GetTick());
        return;
    }

    s_ctx.sweep_result.f0_hz = best_f0;
    s_ctx.sweep_result.f1_hz = best_f1;
    s_ctx.cur_freq.f0_hz     = best_f0;
    s_ctx.cur_freq.f1_hz     = best_f1;

    /* ---- 扫频完成 → 进入预链接状态（协议 §4.2）----
     * 已选出最佳频对 (f0, f1)，后续工作（协议 §4.2.1 自载波 1010 回显、
     * §4.2.3 等 ACK、§4.2.3 超时回退）暂未实现，先停在 PRE_LINKED 状态。 */
    _state_transition(NODE_PRE_LINKED, HAL_GetTick());
}

void app_node_fsm_on_ack_received(uint16_t f0, uint16_t f1)
{
    if (s_ctx.state != NODE_SCAN) {
        return;
    }

    s_ctx.cur_freq.f0_hz = f0;
    s_ctx.cur_freq.f1_hz = f1;

    _state_transition(NODE_LINKED, HAL_GetTick());
}

void app_node_fsm_flag_set(node_xact_bit_t bit)
{
    switch (bit) {
        case XACT_RX_UP_OK:  s_ctx.flags.flag_rx_up_ok  = 1; break;
        case XACT_FWD_DN_OK: s_ctx.flags.flag_fwd_dn_ok = 1; break;
        case XACT_RX_DN_OK:  s_ctx.flags.flag_rx_dn_ok  = 1; break;
        case XACT_FWD_UP_OK: s_ctx.flags.flag_fwd_up_ok = 1; break;
    }
    s_ctx.quiet_start_tick = 0;
}

void app_node_fsm_flag_clear(node_xact_bit_t bit)
{
    switch (bit) {
        case XACT_RX_UP_OK:  s_ctx.flags.flag_rx_up_ok  = 0; break;
        case XACT_FWD_DN_OK: s_ctx.flags.flag_fwd_dn_ok = 0; break;
        case XACT_RX_DN_OK:  s_ctx.flags.flag_rx_dn_ok  = 0; break;
        case XACT_FWD_UP_OK: s_ctx.flags.flag_fwd_up_ok = 0; break;
    }
}

void app_node_fsm_on_quiet_timeout(uint32_t now_tick)
{
    if (app_node_fsm_is_clean()) {
        s_ctx.quiet_start_tick = now_tick;
        _state_transition(NODE_SLEEP, now_tick);
    } else {
        _state_transition(NODE_SCAN_LISTEN, now_tick);
    }
}

void app_node_fsm_on_fault(uint32_t fault_code)
{
    (void)fault_code;
    _state_transition(NODE_FAULT, HAL_GetTick());
}

void app_node_fsm_on_wdt_timeout(void)
{
    _state_transition(NODE_FAULT, HAL_GetTick());
}
