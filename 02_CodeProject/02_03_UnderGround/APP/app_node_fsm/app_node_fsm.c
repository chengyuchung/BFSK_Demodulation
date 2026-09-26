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
#include "bsw_log.h"
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
    /* 起点下方：直接判无效 */
    if (f_hz < (uint16_t)(SWEEP_FREQ_START - SWEEP_FREQ_TOLERANCE_HZ)) {
        return 0xFF;
    }

    /* 算 delta 与候选索引（向下取整），再算余数做就近调整 */
    int32_t delta = (int32_t)f_hz - (int32_t)SWEEP_FREQ_START;
    if (delta < 0) {
        delta = 0;
    }

    int32_t idx     = delta / (int32_t)SWEEP_FREQ_STEP;
    int32_t offset  = delta % (int32_t)SWEEP_FREQ_STEP;

    /* 就近原则：余数 > 步进一半时上取整 */
    if (offset > (int32_t)(SWEEP_FREQ_STEP / 2)) {
        ++idx;
        offset = (int32_t)SWEEP_FREQ_STEP - offset;
    }

    /* 索引越界（最高频点也超出） */
    if (idx < 0 || idx >= (int32_t)SWEEP_FREQ_COUNT) {
        return 0xFF;
    }

    /* 容差检查：offset 超过 ±15 Hz 视为无效 */
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
 * @brief   内部状态转移（仅本文件调用）
 * @note    每次转移同步记录 state_enter_tick
 */
static void _state_transition(node_state_t next_state, uint32_t now_tick)
{
    if (s_ctx.state == next_state) {
        return;   /* 幂等：已经在目标状态 */
    }

    bsw_log("[FSM] %u -> %u (tick=%lu)\r\n",
            (unsigned)s_ctx.state,
            (unsigned)next_state,
            (unsigned long)now_tick);

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
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);    /* 防止 BOOT 残留 */
            bsw_log("[FSM] -> BOOT: ADC off, TIM1 IC off\r\n");
            break;

        case NODE_SCAN_LISTEN:
            bsw_adc_ringbuf_enable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);    /* 监听阶段不解调 */
            /* 新一轮扫频监听：清空 amp_table 与倒计时起点 */
            memset(s_ctx.sweep_amp_table, 0, sizeof(s_ctx.sweep_amp_table));
            s_ctx.sweep_start_tick   = 0;
            s_ctx.sweep_timeout_tick = 0;
            bsw_log("[FSM] -> SCAN_LISTEN: ADC ringbuf enabled (10 kHz), TIM1 IC off, amp_table cleared\r\n");
            break;

        case NODE_SCAN:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            /* 850 ms 倒计时接力继承自 SCAN_LISTEN 阶段首次命中时刻，
             * 不重新设 now_tick，避免双重倒计时 */
            if (s_ctx.sweep_start_tick == 0) {
                /* 兜底：万一从其他状态直跳过来没设过起点，按 now_tick 起算 */
                s_ctx.sweep_start_tick   = now_tick;
                s_ctx.sweep_timeout_tick = now_tick + 850U;
            }
            bsw_log("[FSM] -> SCAN: ADC off, TIM1 IC off\r\n");
            break;

        case NODE_LINKED:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_start(MCAL_TIMER_TIM1);
            bsw_log("[FSM] -> LINKED: freq pair locked (f0=%u, f1=%u), TIM1 IC started\r\n",
                    (unsigned)s_ctx.cur_freq.f0_hz,
                    (unsigned)s_ctx.cur_freq.f1_hz);
            break;

        case NODE_SLEEP:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            bsw_log("[FSM] -> SLEEP: ADC off, TIM1 IC off\r\n");
            break;

        case NODE_PRE_LINKED:
            /* ADC 在 SCAN_LISTEN 已开启，PRE_LINKED 接力继续采（10 kHz）
             * 启动 40 ms 节拍器：从 now_tick 起算，run() 里每 40 ms 检测一次总线能量
             * 若总线已有应答 → 丢弃 f0/f1 → 回 SCAN_LISTEN */
            s_ctx.pre_linked_last_check_tick = now_tick;
            bsw_log("[FSM] -> PRE_LINKED: ADC ringbuf running, 40ms reply monitor armed (f0=%u, f1=%u)\r\n",
                    (unsigned)s_ctx.cur_freq.f0_hz,
                    (unsigned)s_ctx.cur_freq.f1_hz);
            break;

        case NODE_FAULT:
            bsw_adc_ringbuf_disable();
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            bsw_log("[FSM] -> FAULT: fault_code=%lu, ADC off, TIM1 IC off\r\n",
                    (unsigned long)s_ctx.state_enter_tick);
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
    /* 静默判据：连续 3 秒（3000 ms）无高于底噪的有效信号 */
    return (s_ctx.quiet_start_tick > 0)
           && ((now_tick - s_ctx.quiet_start_tick) >= 3000U);
}

/* ========== 公共 API 实现 ========== */

void app_node_fsm_init(uint8_t hard_id)
{
    memset(&s_ctx, 0, sizeof(s_ctx));

    s_ctx.hard_id = hard_id;
    /* pending_buffer/padding 由 memset 清零 */

    /* 把本机硬 ID 注入 BSW 层（地址模块独立持有，后续槽时序依赖此值） */
    bsw_node_id_init(hard_id);

    bsw_log("[FSM] init (hard_id=%u)\r\n", (unsigned)hard_id);

    /* 初始化完成后立即进入扫频监听态，等待上级扫频 */
    s_ctx.state_enter_tick = HAL_GetTick();
    _state_transition(NODE_SCAN_LISTEN, s_ctx.state_enter_tick);
}

void app_node_fsm_run(uint32_t now_tick)
{
    switch (s_ctx.state) {

        case NODE_BOOT:
            /* BOOT 仅用于 init 过渡，正常情况下瞬时离开 */
            break;

        /* -------------------------------------------------------- */
        case NODE_SCAN_LISTEN:
            /* 持续 ADC 采样 + 扫频检测算法每 10 ms 调一次
             * app_node_fsm_on_sweep_detected(f, amp) 把结果写入 amp_table。
             *
             * 850 ms 会话硬超时检查（协议 §二 机制 A）：
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
                bsw_log("[FSM] SCAN_LISTEN: 850 ms timeout, valid_freq_count=%u\r\n",
                        (unsigned)cnt);
                /* 重置倒计时起点，等下一轮扫频；
                 * amp_table 在 on_sweep_complete 失败回滚时被 SCAN_LISTEN
                 * entry action 清空。 */
                s_ctx.sweep_start_tick   = 0;
                s_ctx.sweep_timeout_tick = 0;
                app_node_fsm_on_sweep_complete(cnt);
            }
            break;

        /* -------------------------------------------------------- */
        case NODE_SCAN: {
            /* 扫频进行中：每 10 ms 有一次滑动窗口更新，
             * 调 app_node_fsm_on_sweep_detected() 注入命中频点，
             * 填入 s_ctx.sweep_amp_table。 */

            /* 850 ms 会话硬超时检查（协议 §二 机制 A） */
            if ((now_tick - s_ctx.sweep_start_tick) >= 850U) {
                /* 定时器到期，强制结算成绩单 */
                uint8_t cnt = 0;
                for (int i = 0; i < SWEEP_FREQ_COUNT; ++i) {
                    if (s_ctx.sweep_amp_table[i] > 0) {
                        ++cnt;
                    }
                }
                bsw_log("[FSM] SCAN timeout! valid_freq_count=%u\r\n", (unsigned)cnt);
                app_node_fsm_on_sweep_complete(cnt);
            }
            break;
        }

        /* -------------------------------------------------------- */
        case NODE_PRE_LINKED: {
            /* 协议 §4.2：等待我自己的槽点（my_id × 200 ms），期间监听总线是否有应答
             * 若有人在我之前回发 1010 → 不是给我的扫频 → 丢弃 f0/f1 → 回 SCAN_LISTEN */
            if ((now_tick - s_ctx.pre_linked_last_check_tick) >= 40U) {
                s_ctx.pre_linked_last_check_tick = now_tick;
                uint32_t rms_sq = bsw_adc_ringbuf_calc_rms_sq();
                if (rms_sq > BSW_ADC_RINGBUF_SIGNAL_THRESHOLD) {
                    /* 有人应答 → 这轮扫频不是给我的 */
                    s_ctx.cur_freq.f0_hz     = 0;
                    s_ctx.cur_freq.f1_hz     = 0;
                    s_ctx.sweep_result.f0_hz = 0;
                    s_ctx.sweep_result.f1_hz = 0;
                    bsw_log("[FSM] PRE_LINKED: reply detected (rms^2=%lu > %lu), discard f0/f1, -> SCAN_LISTEN\r\n",
                            (unsigned long)rms_sq,
                            (unsigned long)BSW_ADC_RINGBUF_SIGNAL_THRESHOLD);
                    _state_transition(NODE_SCAN_LISTEN, now_tick);
                }
            }
            break;
        }

        /* -------------------------------------------------------- */
        case NODE_LINKED: {
            /* 工作态：4 标志位驱动断链检测。
             * 若链路断开（总线静默超时），退回 SCAN_LISTEN 重新扫频。 */
            if (s_ctx.flags.flag_fwd_up_ok == 0   /* 未在等待上行 ACK */
                && s_ctx.flags.flag_fwd_dn_ok == 0) { /* 未在等待下行 ACK */
                if (_is_bus_quiet(now_tick)) {
                    bsw_log("[FSM] LINKED: bus quiet 3s, return to SCAN_LISTEN\r\n");
                    _state_transition(NODE_SCAN_LISTEN, now_tick);
                }
            }
            break;
        }

        /* -------------------------------------------------------- */
        case NODE_SLEEP: {
            /* 间歇休眠：每 100 ms 快速听一次能量（唤醒检测），
             * 由 app_task 单独实现唤醒检测逻辑，
             * 唤醒后调 app_node_fsm_on_sweep_detected() 或直接切回 SCAN_LISTEN。 */

            /* 清洁态持续检测：4 标志位任一非零则强制退回 SCAN_LISTEN */
            if (!app_node_fsm_is_clean()) {
                bsw_log("[FSM] SLEEP: dirty flag detected, back to SCAN_LISTEN\r\n");
                _state_transition(NODE_SCAN_LISTEN, now_tick);
            }
            break;
        }

        /* -------------------------------------------------------- */
        case NODE_FAULT:
            /* 故障态：停止一切外设动作，不自动恢复。
             * 等待外部看门狗复位或人工干预。 */
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
    /* 仅在扫频监听态接收检测结果：填表 + 启动/维持 850 ms 倒计时
     * 注意：调到这里不代表"扫频结束"，而是"本 10 ms 窗口解出了一个有效频点"。
     * FSM 不会因为这次调用退出 SCAN_LISTEN——填表工作会持续到 850 ms 到期。 */
    if (s_ctx.state != NODE_SCAN_LISTEN) {
        return;
    }

    /* 1) 就近映射到 amp_table 索引（协议 §三.2 容差 ±15 Hz） */
    uint8_t idx = _sweep_freq_to_index(f_hz);
    if (idx == 0xFF) {
        /* 落在空白带 / 工频带 / 容差外 → 静默丢弃 */
        return;
    }

    /* 2) 只记录不淘汰（协议 §三.3） */
    uint16_t old_amp = s_ctx.sweep_amp_table[idx];
    _update_amp_table(idx, amplitude);

    /* 3) 首次命中 → 启动 850 ms 会话倒计时（协议 §二 机制 A）
     *    后续命中不再重置起点（避免频点密集命中把倒计时一直延后） */
    if (s_ctx.sweep_start_tick == 0) {
        s_ctx.sweep_start_tick   = now_tick;
        s_ctx.sweep_timeout_tick = now_tick + 850U;
    }

    /* 4) 任何有效命中都说明总线非静默 */
    s_ctx.quiet_start_tick = 0;

    /* 仅在 amp_table 实际更新时打印日志，避免重复 10 ms 风暴 */
    if (s_ctx.sweep_amp_table[idx] != old_amp) {
        bsw_log("[FSM] sweep hit: idx=%u, f=%u Hz, amp=%u (table[%u]=%u)\r\n",
                (unsigned)idx, (unsigned)f_hz, (unsigned)amplitude,
                (unsigned)idx, (unsigned)s_ctx.sweep_amp_table[idx]);
    }
}

void app_node_fsm_on_sweep_complete(uint8_t valid_freq_count)
{
    /* ---- 协议 §二 机制 B：有效非零频点 ≥ 5 才确认是真扫频 ---- */
    if (valid_freq_count < 5) {
        bsw_log("[FSM] sweep: valid_count=%u < 5, ignore (noise/collision)\r\n",
                (unsigned)valid_freq_count);
        _state_transition(NODE_SCAN_LISTEN, HAL_GetTick());
        return;
    }

    /* ---- 从 19 格成绩单中决选最优频对 ----
     * 选幅值最大且 |f1-f0| ≥ 150 Hz 的两个频点（协议 §三）
     * TODO: 实际决选算法后续在 bsw_sweep_decision.c 中实现，
     *       此处先取最大值（占位）。 */
    uint16_t best_f0 = 0, best_f1 = 0;
    uint16_t best_amp = 0;
    for (int i = 0; i < SWEEP_FREQ_COUNT; ++i) {
        if (s_ctx.sweep_amp_table[i] > best_amp) {
            best_amp = s_ctx.sweep_amp_table[i];
            best_f0 = SWEEP_FREQ_START + i * SWEEP_FREQ_STEP;
        }
    }
    /* 占位：best_f1 暂时取 best_f0 + 200 Hz（后续决选逻辑补全） */
    best_f1 = best_f0 + 200U;

    s_ctx.sweep_result.f0_hz = best_f0;
    s_ctx.sweep_result.f1_hz = best_f1;
    s_ctx.cur_freq.f0_hz     = best_f0;
    s_ctx.cur_freq.f1_hz     = best_f1;

    bsw_log("[FSM] sweep complete: f0=%u, f1=%u, valid_count=%u\r\n",
            (unsigned)best_f0, (unsigned)best_f1, (unsigned)valid_freq_count);

    /* ---- 扫频完成 → 进入预链接状态（协议 §4.2）----
     * 已选出最佳频对 (f0, f1)，后续工作（协议 §4.2.1 自载波 1010 回显、
     * §4.2.3 等 ACK、§4.2.3 超时回退）暂未实现，先停在 PRE_LINKED 状态。
     * 注：当前不写入 AD9833，1010 回显待补。 */
    _state_transition(NODE_PRE_LINKED, HAL_GetTick());
}

void app_node_fsm_on_ack_received(uint16_t f0, uint16_t f1)
{
    if (s_ctx.state != NODE_SCAN) {
        return;
    }

    s_ctx.cur_freq.f0_hz = f0;
    s_ctx.cur_freq.f1_hz = f1;

    bsw_log("[FSM] ACK received: f0=%u, f1=%u, LOCKED\r\n",
            (unsigned)f0, (unsigned)f1);

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
    /* 任意标志位置位 → 退出清洁态，清除静默计时器 */
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
    /* 协议 §6.2：总线静默超时，进入间歇休眠
     * 仅在清洁态（4 标志位全 0）才允许休眠 */
    if (app_node_fsm_is_clean()) {
        bsw_log("[FSM] quiet timeout 3s, clean -> SLEEP\r\n");
        s_ctx.quiet_start_tick = now_tick;
        _state_transition(NODE_SLEEP, now_tick);
    } else {
        bsw_log("[FSM] quiet timeout 3s, dirty -> SCAN_LISTEN\r\n");
        _state_transition(NODE_SCAN_LISTEN, now_tick);
    }
}

void app_node_fsm_on_fault(uint32_t fault_code)
{
    bsw_log("[FSM] FAULT! code=%lu\r\n", (unsigned long)fault_code);
    _state_transition(NODE_FAULT, HAL_GetTick());
}

void app_node_fsm_on_wdt_timeout(void)
{
    bsw_log("[FSM] WDT timeout -> FAULT\r\n");
    _state_transition(NODE_FAULT, HAL_GetTick());
}
