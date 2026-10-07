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

#define BFSK_DEMOD_TIMEOUT_MS       5000U   /* BFSK 解调器接收超时（毫秒）*/
#define SWEEP_AMP_VALID_THRESHOLD    0U     /* 幅度表有效阈值（>0 表示有信号）*/
#define SWEEP_NOT_STARTED        0U      /* 扫频未启动标志：sweep_start_tick == 0 */
#define PRE_LINKED_CHECK_INTERVAL_MS 40U    /* PRE_LINKED 状态幅度表检查周期（毫秒）*/
#define SLOT_NEVER_ARRIVE        0xFFFFFFFFU  /* 槽点永不到达（未初始化） */
#define LINK_NONE                0U      /* 无活动链路（初始化状态）*/

#include "app_node_fsm.h"

#include <string.h>             /* memset */

#include "main.h"                /* HAL_GetTick, Error_Handler */
#include "bsw_adc_ringbuf.h"
#include "bsw_ad9833.h"
#include "bsw_node_id.h"            /* 节点地址模块：本机 hard_id 注入 */
#include "mcal_timer.h"          /* mcal_timer_ic_start/stop，FSM 直接控制 TIM1 输入捕获 */
#include "bsw_bfsk_demod.h"      /* BFSK 解调器 */
#include "bsw_proto.h"           /* 协议层：帧解析 */
#include "bsw_led.h"             /* LED 状态指示 */
#include "bsw_freq_storage.h"    /* Flash 频点存储 */
#include "bsw_version.h"         /* 版本管理 */

/* ========== 静态全局上下文 ========== */
/* C 标准保证 static 存储期对象零初始化，无需 = {0}，避免与枚举混用的告警 */
static app_node_ctx_t s_ctx;

/* ========== 前向声明 ========== */
static void _state_entry_boot(uint32_t now_tick);
static void _state_entry_scan_listen(uint32_t now_tick);
static void _state_entry_scan(uint32_t now_tick);
static void _state_entry_scan_wait_reply(uint32_t now_tick);
static void _state_entry_pre_linked(uint32_t now_tick);
static void _state_entry_linked(uint32_t now_tick);
static void _state_entry_sleep(uint32_t now_tick);
static void _state_entry_fault(uint32_t now_tick);

static void _state_run_boot(uint32_t now_tick);
static void _state_run_scan_listen(uint32_t now_tick);
static void _state_run_scan(uint32_t now_tick);
static void _state_run_scan_wait_reply(uint32_t now_tick);
static void _state_run_pre_linked(uint32_t now_tick);
static void _state_run_linked(uint32_t now_tick);
static void _state_run_sleep(uint32_t now_tick);
static void _state_run_fault(uint32_t now_tick);

static void _phase_quiet_observe(uint32_t now_tick);
static void _phase_self_reply(uint32_t now_tick);
static void _phase_wait_ack(uint32_t now_tick);

/* ========== 私有辅助 ========== */

/**
 * @brief   频点 → amp_table 索引的"就近映射"（协议 §三.2）
 * @param   f_hz  扫频检测算法解出的瞬时频率
 * @return  0~18 合法索引；SWEEP_FREQ_INDEX_INVALID 表示落在空白带 / 工频带 / 容差外，应丢弃
 * @note    19 个标称频点：125, 175, 225, ..., 1025 Hz（步进 50 Hz）。
 *          容差 ±15 Hz：超过这个窗口的频点视为无效，不写入 amp_table。
 */
static uint8_t _sweep_freq_to_index(uint16_t f_hz)
{
    if (f_hz < (uint16_t)(SWEEP_FREQ_START - SWEEP_FREQ_TOLERANCE_HZ)) {
        return SWEEP_FREQ_INDEX_INVALID;
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
        return SWEEP_FREQ_INDEX_INVALID;
    }

    if (offset > (int32_t)SWEEP_FREQ_TOLERANCE_HZ) {
        return SWEEP_FREQ_INDEX_INVALID;
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
    if (valid_cnt < SWEEP_MIN_VALID_FREQ_CNT) {
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

    uint8_t  f1_idx       = SWEEP_FREQ_INDEX_INVALID;
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
    if (f1_idx == SWEEP_FREQ_INDEX_INVALID) {
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

    /* 调用对应状态的 entry 函数 */
    typedef void (*state_entry_t)(uint32_t);
    static const state_entry_t s_entry_table[] = {
        [NODE_BOOT]             = _state_entry_boot,
        [NODE_SCAN_LISTEN]      = _state_entry_scan_listen,
        [NODE_SCAN]             = _state_entry_scan,
        [NODE_SCAN_WAIT_REPLY]  = _state_entry_scan_wait_reply,
        [NODE_PRE_LINKED]       = _state_entry_pre_linked,
        [NODE_LINKED]           = _state_entry_linked,
        [NODE_SLEEP]            = _state_entry_sleep,
        [NODE_FAULT]            = _state_entry_fault,
    };

    if (next_state < sizeof(s_entry_table) / sizeof(s_entry_table[0])) {
        state_entry_t entry_fn = s_entry_table[next_state];
        if (entry_fn != NULL) {
            entry_fn(now_tick);
        }
    }
}

/**
 * @brief   判断总线是否静默超时（清洁态鉴别）
 * @note    协议 §6.2：总线静默后先等 3~5 s，再降级休眠
 */
static bool _is_bus_quiet(uint32_t now_tick)
{
    return (s_ctx.quiet_start_tick != QUIET_NOT_STARTED)
           && ((now_tick - s_ctx.quiet_start_tick) >= BUS_QUIET_TIMEOUT_MS);
}

/* ========== 状态进入函数（Entry Actions） ========== 
 *
 * ADC / TIM6（采样） vs TIM1 IC（解调） vs AD9833（DDS 发送）的状态归属：
 *
 *   | 状态          | ADC ringbuf | TIM6 | TIM1 IC | AD9833  | 用途                              |
 *   |---------------|-------------|------|---------|---------|-----------------------------------|
 *   | BOOT          | disable     | stop | stop    | sleep   | 时钟/外设初始化                   |
 *   | SCAN_LISTEN   | enable      | run  | stop    | sleep   | 被动监听：填 19 格 amp_table      |
 *   | SCAN          | disable     | stop | stop    | active  | 主动扫频：发送 19 个频点 × 40ms   |
 *   | PRE_LINKED    | enable      | run  | stop    | active  | PHASE1 监测总线，PHASE2 发 1010   |
 *   | LINKED        | disable     | stop | start   | sleep   | BFSK 帧收发（解调器工作）         |
 *   | SLEEP         | disable     | stop | stop    | sleep   | 间歇休眠                          |
 *   | FAULT         | disable     | stop | stop    | sleep   | 故障态静默                        |
 */

static void _state_entry_boot(uint32_t now_tick)
{
    (void)now_tick;
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    bsw_led_set_state(LED_STATE_ALL_OFF);
}

static void _state_entry_scan_listen(uint32_t now_tick)
{
    (void)now_tick;
    bsw_adc_ringbuf_enable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    memset(s_ctx.sweep_amp_table, 0, sizeof(s_ctx.sweep_amp_table));
    s_ctx.sweep_start_tick   = 0;
    s_ctx.sweep_timeout_tick = 0;
    bsw_led_set_state(LED_STATE_LISTEN);
}

static void _state_entry_scan(uint32_t now_tick)
{
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    /* 850 ms 倒计时接力继承自 SCAN_LISTEN 阶段首次命中时刻 */
    if (s_ctx.sweep_start_tick == SWEEP_NOT_STARTED) {
        s_ctx.sweep_start_tick   = now_tick;
        s_ctx.sweep_timeout_tick = now_tick + SWEEP_SESSION_TIMEOUT_MS;
    }
    
    /* 初始化扫频发送状态 */
    s_ctx.scan_freq_index     = 0;  /* 从第一个频点开始（125 Hz） */
    s_ctx.scan_freq_next_tick = now_tick + SCAN_FREQ_DURATION_MS;
    
    /* 配置 AD9833 发送第一个频点 */
    uint16_t first_freq = SWEEP_FREQ_START;  /* 125 Hz */
    (void)bsw_ad9833_sleep(false);  /* 确保唤醒 */
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, first_freq);
    (void)bsw_ad9833_select(BSW_AD9833_REG_0);
    
    bsw_led_set_state(LED_STATE_SCAN);
}

static void _state_entry_scan_wait_reply(uint32_t now_tick)
{
    /* 上级节点扫频完成，等待下级节点的 1010 回应波形
     * 1. 关闭 AD9833（不再发送）
     * 2. 启动 ADC 监听（复用扫频检测算法）
     * 3. 初始化 1010 交替检测状态
     * 
     * 注意：这里不需要启动 TIM1 输入捕获！
     * - TIM1 是 BFSK 解调器专用（半周期解调法）
     * - 我们这里只是检测 1010 波形的频率，用扫频检测算法（ADC + 频谱分析）即可
     * - 外部会调用 app_node_fsm_on_sweep_detected() 注入频率检测结果
     */
    (void)bsw_ad9833_sleep(true);
    bsw_adc_ringbuf_enable();
    /* 不启动 TIM1 输入捕获 - 我们不需要解调 BFSK，只需要频率检测 */
    
    /* 清空幅度表（虽然在这个状态下不使用 amp_table，但保持一致性）*/
    memset(s_ctx.sweep_amp_table, 0, sizeof(s_ctx.sweep_amp_table));
    
    /* 初始化 1010 交替检测状态 - PHASE1: 等待 1010 波形 */
    s_ctx.scan_wait_reply_start_tick = now_tick;
    s_ctx.scan_wait_reply_phase      = SCAN_WAIT_1010;
    s_ctx.scan_ack_sent_tick         = QUIET_NOT_STARTED;
    s_ctx.scan_ack_retry_count       = 0;
    s_ctx.scan_1010_detected_tick    = QUIET_NOT_STARTED;
    s_ctx.reply_last_freq            = FREQ_NOT_LOCKED;
    s_ctx.reply_detected_f0          = FREQ_NOT_LOCKED;
    s_ctx.reply_detected_f1          = FREQ_NOT_LOCKED;
    s_ctx.reply_toggle_count         = 0;
    
    bsw_led_set_state(LED_STATE_LISTEN);
}

static void _state_entry_pre_linked(uint32_t now_tick)
{
    bsw_adc_ringbuf_enable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    s_ctx.pre_linked_last_check_tick = now_tick;
    s_ctx.pre_linked_phase           = PRE_QUIET_OBSERVE;
    s_ctx.pre_phase_enter_tick       = now_tick;
    s_ctx.self_reply_cur_is_f0       = true;
    s_ctx.pre_ack_retry_count        = 0;
    bsw_led_set_state(LED_STATE_PRELINK);
}

static void _state_entry_linked(uint32_t now_tick)
{
    (void)now_tick;
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_start(MCAL_TIMER_TIM1);
    
    proto_rx_init(s_ctx.hard_id);
    
    /* 根据活动链路选择频点 */
    node_freq_pair_t *active_freq = (s_ctx.active_link_is_uplink == LINK_UPLINK)
                                     ? &s_ctx.uplink_freq 
                                     : &s_ctx.downlink_freq;
    
    if (active_freq->f0_hz != FREQ_NOT_LOCKED && active_freq->f1_hz != FREQ_NOT_LOCKED) {
        bsw_bfsk_demod_start(active_freq->f0_hz,
                            active_freq->f1_hz,
                            PRE_BFSK_BIT_PERIOD_MS,
                            BFSK_DEMOD_TIMEOUT_MS,
                            s_ctx.hard_id);
    }
    bsw_led_set_state(LED_STATE_LINKED);
}

static void _state_entry_sleep(uint32_t now_tick)
{
    (void)now_tick;
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    bsw_led_set_state(LED_STATE_ALL_OFF);
}

static void _state_entry_fault(uint32_t now_tick)
{
    (void)now_tick;
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    bsw_led_set_state(LED_STATE_FAULT);
}

/* ========== PRE_LINKED 子状态机 ========== */

typedef void (*phase_handler_t)(uint32_t now_tick);

static const phase_handler_t s_phase_handlers[] = {
    [PRE_QUIET_OBSERVE] = _phase_quiet_observe,
    [PRE_SELF_REPLY]    = _phase_self_reply,
    [PRE_WAIT_ACK]      = _phase_wait_ack,
};

/* PHASE1：等自己槽点（my_id × 200ms），期间 40ms 节拍器测总线避让他人回发 */
static void _phase_quiet_observe(uint32_t now_tick)
{
    /* 每 40ms 检测一次总线能量（ADC RMS²）
     * 若检测到应答信号（别人抢先回发）→ 退让，重新扫频 */
    if ((now_tick - s_ctx.pre_linked_last_check_tick) >= PRE_LINKED_CHECK_INTERVAL_MS) {
        s_ctx.pre_linked_last_check_tick = now_tick;
        uint32_t rms_sq = bsw_adc_ringbuf_calc_rms_sq();
        if (rms_sq > BSW_ADC_RINGBUF_SIGNAL_THRESHOLD) {
            /* 检测到别人应答 → 清空频点，退让重新扫频
             * 注：enable() 已在 PRE_LINKED entry 调用，此处无需重复启动 ADC */
            s_ctx.uplink_freq.f0_hz  = FREQ_NOT_LOCKED;
            s_ctx.uplink_freq.f1_hz  = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f0_hz = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f1_hz = FREQ_NOT_LOCKED;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
            return;
        }
    }

    uint32_t my_slot_offset = bsw_node_id_get_reply_delay_ms();
    if (my_slot_offset == BSW_NODE_ID_SLOT_INVALID) {
        my_slot_offset = PRE_QUIET_OBSERVE_MAX_MS;
    }

    uint32_t since_state_enter = (uint32_t)(now_tick - s_ctx.state_enter_tick);
    if (since_state_enter < my_slot_offset
        && since_state_enter < PRE_QUIET_OBSERVE_MAX_MS) {
        return;
    }

    /* 槽点到期，开始自己回发频对（PHASE2）
     * 先停 ADC（不再需要监测），再配置 AD9833 */
    bsw_adc_ringbuf_disable();
    
    /* PRE_LINKED 状态使用上级频点（被扫方与上级通信） */
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, s_ctx.uplink_freq.f0_hz);
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_1, s_ctx.uplink_freq.f1_hz);
    (void)bsw_ad9833_select(BSW_AD9833_REG_0);

    s_ctx.pre_linked_phase         = PRE_SELF_REPLY;
    s_ctx.pre_phase_enter_tick     = now_tick;
    s_ctx.self_reply_cur_is_f0     = true;
    s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
}

/* PHASE2：在 (f0, f1) 上交替 BFSK 调制持续 2s */
static void _phase_self_reply(uint32_t now_tick)
{
    if ((now_tick - s_ctx.pre_phase_enter_tick) >= PRE_REPLY_DURATION_MS) {
        /* 2s 发送完毕，进入 PHASE3 等待 ACK */
        (void)bsw_ad9833_sleep(true);
        
        /* 启动 BFSK 解调器监听井上 ACK（使用上级频点） */
        bsw_bfsk_demod_start(s_ctx.uplink_freq.f0_hz,
                            s_ctx.uplink_freq.f1_hz,
                            PRE_BFSK_BIT_PERIOD_MS,
                            PRE_ACK_TIMEOUT_MS,     /* 5s 超时 */
                            s_ctx.hard_id);
        
        s_ctx.pre_linked_phase     = PRE_WAIT_ACK;
        s_ctx.pre_phase_enter_tick = now_tick;
        return;
    }
    if ((int32_t)(now_tick - s_ctx.self_reply_next_bit_tick) < 0) {
        return;
    }
    if (s_ctx.self_reply_cur_is_f0) {
        (void)bsw_ad9833_select(BSW_AD9833_REG_1);
        s_ctx.self_reply_cur_is_f0 = false;
    } else {
        (void)bsw_ad9833_select(BSW_AD9833_REG_0);
        s_ctx.self_reply_cur_is_f0 = true;
    }
    s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
}

/* PHASE3：等井上 ACK */
static void _phase_wait_ack(uint32_t now_tick)
{
    bsw_bfsk_demod_state_t demod_state = bsw_bfsk_demod_get_state();
    
    if (demod_state == BSW_BFSK_DEMOD_DONE_OK) {
        /* 收到帧，检查是否为 ACK */
        bsw_bfsk_demod_result_t demod_result;
        if (bsw_bfsk_demod_take_result(&demod_result) == BSW_BFSK_DEMOD_OK) {
            /* 解析帧类型：取 type_info 高 4 位 */
            uint8_t frame_type = (demod_result.frame_type_info >> 4) & 0x0F;
            uint8_t msg_num    = demod_result.frame_type_info & 0x0F;
            
            /* 检查是否为 REPLY 类型 + REPLY_FREQ_PAIR 消息（井上确认频对）
             * PROTO_TYPE_REPLY = 0x3, PROTO_REPLY_FREQ_PAIR = 0x0 */
            if (frame_type == 0x3 && msg_num == 0x0) {
                
                /* 解析载荷：[f0_hz(2)][f1_hz(2)] 小端序 */
                if (demod_result.frame_len >= 4) {
                    uint16_t ack_f0 = demod_result.frame_payload[0] | (demod_result.frame_payload[1] << 8);
                    uint16_t ack_f1 = demod_result.frame_payload[2] | (demod_result.frame_payload[3] << 8);
                    
                    /* 验证频对是否匹配（PRE_LINKED 使用上级频点） */
                    if (ack_f0 == s_ctx.uplink_freq.f0_hz && ack_f1 == s_ctx.uplink_freq.f1_hz) {
                        /* ACK 确认成功，进入 LINKED 工作态 */
                        bsw_bfsk_demod_stop();
                        _state_transition(NODE_LINKED, now_tick);
                        return;
                    }
                }
            }
        }
        
        /* 收到其他帧，继续等待（不重启解调器，继续监听）*/
        return;
    }
    else if (demod_state == BSW_BFSK_DEMOD_DONE_TIMEOUT) {
        /* 超时处理 */
        bsw_bfsk_demod_stop();
        
        if (s_ctx.pre_ack_retry_count < PRE_ACK_MAX_RETRY) {
            /* 还有重试机会，重新回到 PHASE2 发送 1010 */
            s_ctx.pre_ack_retry_count++;
            
            /* 唤醒 AD9833，重新配置频点（使用上级频点） */
            (void)bsw_ad9833_sleep(false);
            (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, s_ctx.uplink_freq.f0_hz);
            (void)bsw_ad9833_set_freq(BSW_AD9833_REG_1, s_ctx.uplink_freq.f1_hz);
            (void)bsw_ad9833_select(BSW_AD9833_REG_0);
            
            s_ctx.pre_linked_phase         = PRE_SELF_REPLY;
            s_ctx.pre_phase_enter_tick     = now_tick;
            s_ctx.self_reply_cur_is_f0     = true;
            s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
        } else {
            /* 重试次数用尽，清空上级频点，重新扫频 */
            s_ctx.uplink_freq.f0_hz  = FREQ_NOT_LOCKED;
            s_ctx.uplink_freq.f1_hz  = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f0_hz = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f1_hz = FREQ_NOT_LOCKED;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
        }
    }
    /* 其他状态（IDLE/RUNNING）继续等待 */
}

/* ========== 状态运行函数（Run Actions） ========== */

static void _state_run_boot(uint32_t now_tick)
{
    (void)now_tick;
    /* BOOT 状态无运行逻辑，等待外部切换 */
}

static void _state_run_scan_listen(uint32_t now_tick)
{
    /* 850 ms 会话硬超时检查（协议 §二 机制 A） */
    if (s_ctx.sweep_start_tick != SWEEP_NOT_STARTED
        && (now_tick - s_ctx.sweep_start_tick) >= SWEEP_SESSION_TIMEOUT_MS) {
        uint8_t cnt = 0;
        for (int i = 0; i < SWEEP_FREQ_COUNT; ++i) {
            if (s_ctx.sweep_amp_table[i] > SWEEP_AMP_VALID_THRESHOLD) {
                ++cnt;
            }
        }
        s_ctx.sweep_start_tick   = SWEEP_NOT_STARTED;
        s_ctx.sweep_timeout_tick = SWEEP_NOT_STARTED;
        app_node_fsm_on_sweep_complete(cnt);
    }
}

static void _state_run_scan(uint32_t now_tick)
{
    /* 检查是否到达下一个频点切换时刻 */
    if ((int32_t)(now_tick - s_ctx.scan_freq_next_tick) >= 0) {
        /* 移动到下一个频点 */
        s_ctx.scan_freq_index++;
        
        if (s_ctx.scan_freq_index < SWEEP_FREQ_COUNT) {
            /* 计算并发送下一个频点 */
            uint16_t next_freq = SWEEP_FREQ_START + (s_ctx.scan_freq_index * SWEEP_FREQ_STEP);
            (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, next_freq);
            (void)bsw_ad9833_select(BSW_AD9833_REG_0);
            
            /* 更新下一次切换时刻 */
            s_ctx.scan_freq_next_tick = now_tick + SCAN_FREQ_DURATION_MS;
        } else {
            /* 19 个频点全部发送完毕，停止发送，进入 AD9833 休眠 */
            (void)bsw_ad9833_sleep(true);
            
            /* 注：扫频发送完成后，不立即退出 SCAN 状态，
             * 而是等待 850ms 硬超时，让接收方有足够时间处理幅度表。
             * 超时后由下面的逻辑触发 on_sweep_complete() */
        }
    }
    
    /* 850 ms 会话硬超时检查 */
    if ((now_tick - s_ctx.sweep_start_tick) >= SWEEP_SESSION_TIMEOUT_MS) {
        /* SCAN 状态（主动发送方）扫频结束
         * 进入 SCAN_WAIT_REPLY 状态，监听下级节点的 1010 回应波形 */
        _state_transition(NODE_SCAN_WAIT_REPLY, now_tick);
    }
}

static void _state_run_scan_wait_reply(uint32_t now_tick)
{
    /* 上级节点等待下级的 1010 回应波形
     * 
     * PHASE1 SCAN_WAIT_1010: 等待下级 1010 波形（ADC 频率检测）
     *   - 检测到有效 1010 → 发送 ACK → 进入 PHASE2
     *   - 超时 → 回到 SCAN_LISTEN
     * 
     * PHASE2 SCAN_ACK_SENT: 已发送 ACK，确认等待（BFSK 解调器监听重复 1010）
     *   - 检测到重复 1010 → ACK 丢失，重发 ACK
     *   - 超时无 1010 → 确认通信建立，进入 LINKED
     *   - 重发次数用尽 → 回到 SCAN_LISTEN
     */
    
    if (s_ctx.scan_wait_reply_phase == SCAN_WAIT_1010) {
        /* PHASE1: 等待 1010 波形 */
        
        /* 超时检查 */
        if ((now_tick - s_ctx.scan_wait_reply_start_tick) >= SCAN_WAIT_REPLY_TIMEOUT_MS) {
            /* 超时未收到下级回应，回到 SCAN_LISTEN 状态 */
            bsw_adc_ringbuf_disable();
            _state_transition(NODE_SCAN_LISTEN, now_tick);
            return;
        }
        
        /* 1010 检测由 app_node_fsm_on_sweep_detected() 处理
         * 检测到有效频对后会自动切换到 PHASE2 */
         
    } else if (s_ctx.scan_wait_reply_phase == SCAN_ACK_SENT) {
        /* PHASE2: ACK 已发送，等待确认 */
        
        /* 超时检查 - 如果超时无重复 1010，说明下级收到了 ACK */
        if ((now_tick - s_ctx.scan_ack_sent_tick) >= SCAN_ACK_CONFIRM_TIMEOUT_MS) {
            /* 确认通信建立成功，进入 LINKED */
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            _state_transition(NODE_LINKED, now_tick);
            return;
        }
        
        /* 继续监听是否有重复的 1010 波形（使用 ADC 频率检测）
         * 注意：这里不使用 BFSK 解调器，因为 1010 只是载波切换，不是数据帧
         * 1010 检测逻辑由 on_sweep_detected 事件注入，通过 reply_toggle_count 判断 */
        
        /* 如果检测到重复的 1010（reply_toggle_count >= REPLY_1010_MIN_TOGGLE_COUNT）
         * 说明下级没收到 ACK，需要重发 */
        if (s_ctx.reply_toggle_count >= REPLY_1010_MIN_TOGGLE_COUNT) {
            /* 检测到重复的 1010 */
            
            /* 检查重发次数 */
            if (s_ctx.scan_ack_retry_count < SCAN_ACK_MAX_RETRY) {
                /* 等待 1010 发送完成（PRE_REPLY_DURATION_MS = 2000ms）
                 * 确保总线空闲后再发送 ACK */
                
                if (s_ctx.scan_1010_detected_tick == QUIET_NOT_STARTED) {
                    /* 首次检测到 1010，记录时刻，开始等待 */
                    s_ctx.scan_1010_detected_tick = now_tick;
                } else if ((now_tick - s_ctx.scan_1010_detected_tick) >= PRE_REPLY_DURATION_MS) {
                    /* 等待完成（2000ms 后），总线空闲，现在可以重发 ACK */
                    s_ctx.scan_ack_retry_count++;
                    
                    /* TODO: 发送 ACK 帧
                     * 载荷：[f0_hz(2)][f1_hz(2)] 小端序 */
                    
                    /* 更新重发时刻 */
                    s_ctx.scan_ack_sent_tick = now_tick;
                    
                    /* 重置检测标志，准备下一次检测 */
                    s_ctx.scan_1010_detected_tick = QUIET_NOT_STARTED;
                    s_ctx.reply_toggle_count = FLAG_CLEARED;
                    s_ctx.reply_last_freq = FREQ_NOT_LOCKED;
                }
                /* else: 继续等待 1010 发送完成（还没到 2000ms）*/
            } else {
                /* 重发次数用尽，清空下级频点，回到 SCAN_LISTEN */
                s_ctx.downlink_freq.f0_hz = FREQ_NOT_LOCKED;
                s_ctx.downlink_freq.f1_hz = FREQ_NOT_LOCKED;
                s_ctx.sweep_result.f0_hz  = FREQ_NOT_LOCKED;
                s_ctx.sweep_result.f1_hz  = FREQ_NOT_LOCKED;
                _state_transition(NODE_SCAN_LISTEN, now_tick);
            }
        }
        /* 其他情况继续等待 */
    }
}

static void _state_run_pre_linked(uint32_t now_tick)
{
    phase_handler_t h = s_phase_handlers[s_ctx.pre_linked_phase];
    if (h != NULL) {
        h(now_tick);
    }
}

/* ========== LINKED 状态帧处理（函数表驱动） ========== */

typedef void (*frame_handler_t)(const proto_frame_t *frame);

/* 帧类型处理函数（待实现） */
static void _handle_query_frame(const proto_frame_t *frame)
{
    uint8_t local_addr = bsw_node_id_get();
    
    /* 地址过滤：不是发给我的就忽略 */
    if (frame->addr.dst != local_addr) {
        return;
    }
    
    /* 根据查询类型处理 */
    switch (frame->type_info.msg_num) {
        case PROTO_QUERY_VERSION:
        {
            const bsw_version_info_t *ver = bsw_version_get_info();
            
            /* 准备回复帧 */
            proto_frame_t reply;
            reply.addr.dst = frame->addr.src;  /* 回复给查询方 */
            reply.addr.src = local_addr;
            reply.type_info.type = PROTO_TYPE_REPLY;
            reply.type_info.msg_num = PROTO_REPLY_VERSION;
            reply.seq = frame->seq;
            reply.payload_len = PROTO_VERSION_PAYLOAD_SIZE;
            
            /* 打包版本号（小端格式）*/
            reply.payload[0] = (uint8_t)((ver->sw_version_date >>  0) & 0xFFU);
            reply.payload[1] = (uint8_t)((ver->sw_version_date >>  8) & 0xFFU);
            reply.payload[2] = (uint8_t)((ver->sw_version_date >> 16) & 0xFFU);
            reply.payload[3] = (uint8_t)((ver->sw_version_date >> 24) & 0xFFU);
            reply.payload[4] = (uint8_t)((ver->hw_version_date >>  0) & 0xFFU);
            reply.payload[5] = (uint8_t)((ver->hw_version_date >>  8) & 0xFFU);
            reply.payload[6] = (uint8_t)((ver->hw_version_date >> 16) & 0xFFU);
            reply.payload[7] = (uint8_t)((ver->hw_version_date >> 24) & 0xFFU);
            
            /* 发送回复帧（通过 proto_frame_pack 和 BFSK 调制）*/
            uint8_t tx_buf[PROTO_FRAME_MAX];
            uint32_t tx_len = 0;
            proto_err_t err = proto_frame_pack(tx_buf, sizeof(tx_buf), &reply, &tx_len);
            
            if (err == PROTO_ERR_OK && tx_len > 0) {
                /* TODO: 调用 BFSK 调制器发送 tx_buf，长度 tx_len
                 * 示例：bsw_bfsk_mod_send(tx_buf, tx_len);
                 * 注意：需要根据 active_link 选择正确的频点对
                 */
            }
            break;
        }
        
        case PROTO_QUERY_TEMP_PRESS:
            /* TODO: 处理温压查询 */
            break;
            
        case PROTO_QUERY_BATTERY:
            /* TODO: 处理电池查询 */
            break;
            
        case PROTO_QUERY_FAULT:
            /* TODO: 处理故障查询 */
            break;
            
        default:
            /* 未知查询类型，忽略 */
            break;
    }
}

static void _handle_control_frame(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 处理 CONTROL 类型帧（休眠/唤醒/阈值设置）*/
}

static void _handle_ask_frame(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 处理 ASK 类型帧（空闲确认）*/
}

static void _handle_reply_frame(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 处理 REPLY 类型帧（频点确认/ACK/NACK/重传请求）*/
}

static void _handle_alarm_frame(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 处理 ALARM 类型帧（低电/温度/压力/传感器故障）*/
}

static void _handle_scan_frame(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 处理 SCAN 类型帧（扫频起始/结束）*/
}

/* 帧类型分发表（索引对齐 proto_type_t 枚举）*/
static const frame_handler_t s_frame_handlers[] = {
    [PROTO_TYPE_QUERY]    = _handle_query_frame,    /* 0x0 */
    [PROTO_TYPE_CONTROL]  = _handle_control_frame,  /* 0x1 */
    [PROTO_TYPE_ASK]      = _handle_ask_frame,      /* 0x2 */
    [PROTO_TYPE_REPLY]    = _handle_reply_frame,    /* 0x3 */
    [PROTO_TYPE_ALARM]    = _handle_alarm_frame,    /* 0x4 */
    [5] = NULL, [6] = NULL, [7] = NULL, [8] = NULL, /* 预留 */
    [9] = NULL, [10] = NULL, [11] = NULL, [12] = NULL,
    [PROTO_TYPE_SCAN]     = _handle_scan_frame,     /* 0xD */
    [14] = NULL,
    [PROTO_TYPE_RESERVED] = NULL,                   /* 0xF */
};

static void _state_run_linked(uint32_t now_tick)
{
    bsw_bfsk_demod_state_t demod_state = bsw_bfsk_demod_get_state();
    
    if (demod_state == BSW_BFSK_DEMOD_DONE_OK) {
        bsw_bfsk_demod_result_t demod_result;
        if (bsw_bfsk_demod_take_result(&demod_result) == BSW_BFSK_DEMOD_OK) {
            proto_frame_t frame;
            frame.addr.dst = (demod_result.frame_addr >> 4) & 0x0F;
            frame.addr.src = demod_result.frame_addr & 0x0F;
            frame.type_info.type = (proto_type_t)((demod_result.frame_type_info >> 4) & 0x0F);
            frame.type_info.msg_num = demod_result.frame_type_info & 0x0F;
            frame.seq = demod_result.frame_seq;
            frame.payload_len = demod_result.frame_len;
            memcpy(frame.payload, demod_result.frame_payload, demod_result.frame_len);
            
            /* TODO: CRC16 校验（可选） */
            
            /* 根据帧类型分发到对应处理函数 */
            if (frame.type_info.type < sizeof(s_frame_handlers) / sizeof(s_frame_handlers[0])) {
                frame_handler_t handler = s_frame_handlers[frame.type_info.type];
                if (handler != NULL) {
                    handler(&frame);
                }
            }
            
            app_node_fsm_flag_set(XACT_RX_UP_OK);
        }
    }
    else if (demod_state == BSW_BFSK_DEMOD_DONE_TIMEOUT) {
        bsw_bfsk_demod_stop();
        /* 根据需要重新启动 */
    }
    
    /* 断链检测 */
    if ((s_ctx.flags.flag_fwd_up_ok == false)
        && (s_ctx.flags.flag_fwd_dn_ok == false)) {
        if (_is_bus_quiet(now_tick)) {
            _state_transition(NODE_SCAN_LISTEN, now_tick);
        }
    }
}

static void _state_run_sleep(uint32_t now_tick)
{
    if (!app_node_fsm_is_clean()) {
        _state_transition(NODE_SCAN_LISTEN, now_tick);
    }
}

static void _state_run_fault(uint32_t now_tick)
{
    (void)now_tick;
    /* FAULT 状态静默，等待外部复位 */
}

/* ========== 公共 API 实现 ========== */

void app_node_fsm_init(uint8_t hard_id)
{
    memset(&s_ctx, 0, sizeof(s_ctx));

    s_ctx.hard_id = hard_id;
    s_ctx.noise_floor = SWEEP_DEFAULT_NOISE_FLOOR;
    
    /* 初始化频点为未锁定状态 */
    s_ctx.uplink_freq.f0_hz = FREQ_NOT_LOCKED;
    s_ctx.uplink_freq.f1_hz = FREQ_NOT_LOCKED;
    s_ctx.downlink_freq.f0_hz = FREQ_NOT_LOCKED;
    s_ctx.downlink_freq.f1_hz = FREQ_NOT_LOCKED;
    
    /* 初始化活动链路为未激活 */
    s_ctx.active_link_is_uplink = LINK_NONE;

    bsw_node_id_init(hard_id);
    bsw_led_init();  /* 初始化 LED 指示灯 */
    bsw_version_init();  /* 初始化版本模块 */
    
    /* 初始化 Flash 频点存储模块 */
    bsw_freq_storage_init();
    
    /* 尝试从 Flash 恢复频点配置（用于快速恢复通信）*/
    uint16_t saved_f0 = 0, saved_f1 = 0;
    if (bsw_freq_storage_load(&saved_f0, &saved_f1) == 0) {
        /* 成功加载备份的频点，保存到上下文
         * 注意：这里不直接进入 LINKED 状态，因为：
         * 1. 对端可能已经掉电重启，频点可能已变化
         * 2. 需要先通过扫频/监听重新建立通信
         * 3. 只是将频点作为"上次锁定频点"供诊断和优化使用
         * 
         * TODO: 未来可以添加"快速恢复"模式：
         * - 优先尝试使用备份频点直接通信
         * - 超时后再回退到扫频/监听流程
         * 
         * 当前策略：假设井下节点主要作为被扫方，将备份频点视为上级频点
         */
        s_ctx.uplink_freq.f0_hz = saved_f0;
        s_ctx.uplink_freq.f1_hz = saved_f1;
    }

    s_ctx.state_enter_tick = HAL_GetTick();
    _state_transition(NODE_SCAN_LISTEN, s_ctx.state_enter_tick);
}

void app_node_fsm_run(uint32_t now_tick)
{
    /* 状态运行函数表 */
    typedef void (*state_run_t)(uint32_t);
    static const state_run_t s_run_table[] = {
        [NODE_BOOT]             = _state_run_boot,
        [NODE_SCAN_LISTEN]      = _state_run_scan_listen,
        [NODE_SCAN]             = _state_run_scan,
        [NODE_SCAN_WAIT_REPLY]  = _state_run_scan_wait_reply,
        [NODE_PRE_LINKED]       = _state_run_pre_linked,
        [NODE_LINKED]           = _state_run_linked,
        [NODE_SLEEP]            = _state_run_sleep,
        [NODE_FAULT]            = _state_run_fault,
    };

    if (s_ctx.state < sizeof(s_run_table) / sizeof(s_run_table[0])) {
        state_run_t run_fn = s_run_table[s_ctx.state];
        if (run_fn != NULL) {
            run_fn(now_tick);
        }
    }
}

/* ---- 查询 API ---- */

node_state_t app_node_fsm_get_state(void)
{
    return s_ctx.state;
}

bool app_node_fsm_is_clean(void)
{
    return (s_ctx.flags.flag_rx_up_ok  == false)
           && (s_ctx.flags.flag_fwd_dn_ok == false)
           && (s_ctx.flags.flag_rx_dn_ok  == false)
           && (s_ctx.flags.flag_fwd_up_ok == false);
}

bool app_node_fsm_is_uplink_locked(void)
{
    return (s_ctx.uplink_freq.f0_hz != FREQ_NOT_LOCKED);
}

bool app_node_fsm_is_downlink_locked(void)
{
    return (s_ctx.downlink_freq.f0_hz != FREQ_NOT_LOCKED);
}

void app_node_fsm_get_uplink_freq(uint16_t *f0_hz, uint16_t *f1_hz)
{
    if (f0_hz) *f0_hz = s_ctx.uplink_freq.f0_hz;
    if (f1_hz) *f1_hz = s_ctx.uplink_freq.f1_hz;
}

void app_node_fsm_get_downlink_freq(uint16_t *f0_hz, uint16_t *f1_hz)
{
    if (f0_hz) *f0_hz = s_ctx.downlink_freq.f0_hz;
    if (f1_hz) *f1_hz = s_ctx.downlink_freq.f1_hz;
}

const app_node_ctx_t *app_node_fsm_get_ctx(void)
{
    return &s_ctx;
}

/* ========== 事件注入实现 ========== */

void app_node_fsm_on_sweep_detected(uint16_t f_hz, uint16_t amplitude, uint32_t now_tick)
{
    /* SCAN_LISTEN 状态：下级节点监听上级的 19 频点扫频 */
    if (s_ctx.state == NODE_SCAN_LISTEN) {
        uint8_t idx = _sweep_freq_to_index(f_hz);
        if (idx == SWEEP_FREQ_INDEX_INVALID) {
            return;
        }

        _update_amp_table(idx, amplitude);

        /* 首次命中 → 启动 850 ms 会话倒计时（协议 §二 机制 A）*/
        if (s_ctx.sweep_start_tick == SWEEP_NOT_STARTED) {
            s_ctx.sweep_start_tick   = now_tick;
            s_ctx.sweep_timeout_tick = now_tick + SWEEP_SESSION_TIMEOUT_MS;
        }

        s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
        return;
    }
    
    /* SCAN_WAIT_REPLY 状态：上级节点监听下级的 1010 交替波形 */
    if (s_ctx.state == NODE_SCAN_WAIT_REPLY) {
        /* 检测 1010 交替模式：f0 ↔ f1 ↔ f0 ↔ f1
         * 策略：记录最近两个不同的频率，检测它们的交替切换 */
        
        /* 忽略振幅过低的信号（可能是噪声）*/
        if (amplitude < SWEEP_AMP_VALID_THRESHOLD + REPLY_MIN_AMPLITUDE_MARGIN) {
            return;
        }
        
        /* 第一次检测到频率 */
        if (s_ctx.reply_detected_f0 == FREQ_NOT_LOCKED) {
            s_ctx.reply_detected_f0 = f_hz;
            s_ctx.reply_last_freq   = f_hz;
            s_ctx.reply_toggle_count = 0;
            return;
        }
        
        /* 检测到与 f0 不同的频率 → 作为 f1 */
        if (s_ctx.reply_detected_f1 == FREQ_NOT_LOCKED) {
            /* 频率差异要足够大（至少 100Hz），避免噪声干扰 */
            int32_t freq_diff = (int32_t)f_hz - (int32_t)s_ctx.reply_detected_f0;
            if (freq_diff < -(int32_t)REPLY_FREQ_DIFF_MIN_HZ || freq_diff > (int32_t)REPLY_FREQ_DIFF_MIN_HZ) {
                s_ctx.reply_detected_f1 = f_hz;
                s_ctx.reply_last_freq   = f_hz;
                s_ctx.reply_toggle_count = FLAG_SET;  /* 第一次切换 */
            }
            return;
        }
        
        /* 已经检测到 f0 和 f1，现在检测交替模式 */
        /* 判断当前频率应该是 f0 还是 f1（容差 ±30Hz）*/
        int32_t diff_to_f0 = (int32_t)f_hz - (int32_t)s_ctx.reply_detected_f0;
        int32_t diff_to_f1 = (int32_t)f_hz - (int32_t)s_ctx.reply_detected_f1;
        
        if (diff_to_f0 < 0) diff_to_f0 = -diff_to_f0;
        if (diff_to_f1 < 0) diff_to_f1 = -diff_to_f1;
        
        uint16_t current_freq;
        if (diff_to_f0 < diff_to_f1 && diff_to_f0 < REPLY_FREQ_TOLERANCE_HZ) {
            current_freq = s_ctx.reply_detected_f0;
        } else if (diff_to_f1 < REPLY_FREQ_TOLERANCE_HZ) {
            current_freq = s_ctx.reply_detected_f1;
        } else {
            /* 频率偏差太大，可能是噪声，重置检测 */
            s_ctx.reply_detected_f0  = FREQ_NOT_LOCKED;
            s_ctx.reply_detected_f1  = FREQ_NOT_LOCKED;
            s_ctx.reply_toggle_count = 0;
            return;
        }
        
        /* 检测到切换（当前频率 != 上次频率）*/
        if (current_freq != s_ctx.reply_last_freq) {
            s_ctx.reply_toggle_count++;
            s_ctx.reply_last_freq = current_freq;
            
            /* 连续检测到 6 次交替切换（f0→f1→f0→f1→f0→f1→f0），确认是 1010 模式 */
            if (s_ctx.reply_toggle_count >= REPLY_1010_MIN_TOGGLE_COUNT) {
                /* 只在 PHASE1 处理 1010 检测 */
                if (s_ctx.scan_wait_reply_phase != SCAN_WAIT_1010) {
                    return;
                }
                
                /* 确定 f0 < f1 */
                uint16_t f0 = s_ctx.reply_detected_f0;
                uint16_t f1 = s_ctx.reply_detected_f1;
                if (f0 > f1) {
                    uint16_t tmp = f0;
                    f0 = f1;
                    f1 = tmp;
                }
                
                /* 验证频率间隔 >= 150Hz */
                if ((f1 - f0) < SWEEP_MIN_FREQ_GAP_HZ) {
                    /* 频率间隔太小，重置检测 */
                    s_ctx.reply_detected_f0  = FREQ_NOT_LOCKED;
                    s_ctx.reply_detected_f1  = FREQ_NOT_LOCKED;
                    s_ctx.reply_toggle_count = 0;
                    return;
                }
                
                /* 成功检测到有效频对！保存到上下文 */
                /* 扫频方：建立的是下级频点（与下级通信） */
                s_ctx.downlink_freq.f0_hz = f0;
                s_ctx.downlink_freq.f1_hz = f1;
                s_ctx.sweep_result.f0_hz = f0;
                s_ctx.sweep_result.f1_hz = f1;
                s_ctx.active_link_is_uplink = LINK_DOWNLINK;
                
                /* 备份频对到 Flash（上级节点确认下级选择的频点）*/
                int32_t ret = bsw_freq_storage_save(f0, f1);
                (void)ret;  /* 备份失败不影响通信建立，只是下次掉电无法快速恢复 */
                
                /* TODO: 发送 ACK 帧（PROTO_TYPE_REPLY + PROTO_REPLY_FREQ_PAIR）
                 * 载荷：[f0_hz(2)][f1_hz(2)] 小端序
                 * 暂时先切换到 PHASE2，等协议层完善后再实现 ACK 发送 */
                
                /* 停止 ADC，启动 BFSK 解调器监听重复的 1010 */
                bsw_adc_ringbuf_disable();
                mcal_timer_ic_start(MCAL_TIMER_TIM1);
                bsw_bfsk_demod_start(f0, f1, PRE_BFSK_BIT_PERIOD_MS, SCAN_ACK_CONFIRM_TIMEOUT_MS, s_ctx.hard_id);
                
                /* 切换到 PHASE2: ACK 已发送，等待确认 */
                s_ctx.scan_wait_reply_phase = SCAN_ACK_SENT;
                s_ctx.scan_ack_sent_tick    = now_tick;
                s_ctx.scan_ack_retry_count  = 0;
            }
        }
        
        return;
    }
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
    
    /* 被扫方：建立的是上级频点（与上级通信） */
    s_ctx.uplink_freq.f0_hz = best_f0;
    s_ctx.uplink_freq.f1_hz = best_f1;
    s_ctx.active_link_is_uplink = LINK_UPLINK;

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

    /* 被扫方收到上级 ACK，确认上级频点 */
    s_ctx.uplink_freq.f0_hz = f0;
    s_ctx.uplink_freq.f1_hz = f1;
    s_ctx.active_link_is_uplink = LINK_UPLINK;

    /* 备份频对到 Flash（下级节点收到上级 ACK，确认频点被接受）*/
    int32_t ret = bsw_freq_storage_save(f0, f1);
    (void)ret;  /* 备份失败不影响进入 LINKED，只是下次掉电无法快速恢复 */

    _state_transition(NODE_LINKED, HAL_GetTick());
}

/* ========== 事务标志位操作 ========== */

void app_node_fsm_flag_set(node_xact_bit_t bit)
{
    switch (bit) {
        case XACT_RX_UP_OK:
            s_ctx.flags.flag_rx_up_ok = FLAG_SET;
            break;
        case XACT_FWD_DN_OK:
            s_ctx.flags.flag_fwd_dn_ok = FLAG_SET;
            break;
        case XACT_RX_DN_OK:
            s_ctx.flags.flag_rx_dn_ok = FLAG_SET;
            break;
        case XACT_FWD_UP_OK:
            s_ctx.flags.flag_fwd_up_ok = FLAG_SET;
            break;
        default:
            break;
    }
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

void app_node_fsm_flag_clear(node_xact_bit_t bit)
{
    switch (bit) {
        case XACT_RX_UP_OK:
            s_ctx.flags.flag_rx_up_ok = FLAG_CLEARED;
            break;
        case XACT_FWD_DN_OK:
            s_ctx.flags.flag_fwd_dn_ok = FLAG_CLEARED;
            break;
        case XACT_RX_DN_OK:
            s_ctx.flags.flag_rx_dn_ok = FLAG_CLEARED;
            break;
        case XACT_FWD_UP_OK:
            s_ctx.flags.flag_fwd_up_ok = FLAG_CLEARED;
            break;
        default:
            break;
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
