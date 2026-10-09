/**
 * @file    app_node_fsm.c
 * @brief   井下节点状态机实现
 */

#define BFSK_DEMOD_TIMEOUT_MS        5000U
#define SWEEP_AMP_VALID_THRESHOLD    0U
#define SWEEP_NOT_STARTED            0U
#define PRE_LINKED_CHECK_INTERVAL_MS 40U
#define SLOT_NEVER_ARRIVE            0xFFFFFFFFU
#define LINK_NONE                    0U

#include "app_node_fsm.h"
#include <string.h>
#include "main.h"
#include "bsw_adc_ringbuf.h"
#include "bsw_ad9833.h"
#include "bsw_node_id.h"
#include "mcal_timer.h"
#include "bsw_bfsk_demod.h"
#include "bsw_proto.h"
#include "bsw_led.h"
#include "bsw_freq_storage.h"
#include "bsw_version.h"
#include "app_frame_handler.h"  /* 帧业务处理层（新模块）*/

/* 静态全局上下文 */
static app_node_ctx_t s_ctx;

/* 内部类型定义 */
typedef enum {
    LINK_OK               = 0,
    LINK_FAIL_NEED_SCAN   = 1,
    LINK_FAIL_NEED_LISTEN = 2,
} link_check_result_t;

/* 前向声明 */
static void _state_transition(node_state_t next_state, uint32_t now_tick);

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

/**
 * @brief   频点 → amp_table 索引映射
 * @param   f_hz  频率 (Hz)
 * @return  0~18 合法索引；SWEEP_FREQ_INDEX_INVALID 表示超出容差
 * @note    19 个频点：125, 175, 225, ..., 1025 Hz，步进 50 Hz，容差 ±15 Hz
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
 * @brief   校验频点是否在频点表内
 * @param   f_hz  待校验的频点 (Hz)
 * @return  true = 在表内，false = 不在
 */
static bool _is_freq_in_table(uint16_t f_hz)
{
    return (_sweep_freq_to_index(f_hz) != SWEEP_FREQ_INDEX_INVALID);
}

/**
 * @brief   根据频点有效性和节点角色选择初始状态
 * @note    上行有效 → LINKED（可被上位机控制）
 *          上行无效 → SCAN_LISTEN（失联，被动监听）
 *          井上节点（无上级）：直接进入 LINKED（串口直连上位机）
 */
static void _select_initial_state(void)
{
    uint32_t now_tick = HAL_GetTick();
    s_ctx.state_enter_tick = now_tick;

    bool uplink_valid   = (s_ctx.uplink_freq.f0_hz != FREQ_NOT_LOCKED 
                           && s_ctx.uplink_freq.f1_hz != FREQ_NOT_LOCKED);

    if (!app_node_fsm_has_uplink()) {
        /* 井上节点：串口直连上位机，永远可控 */
        s_ctx.active_link_is_uplink = LINK_DOWNLINK;
        _state_transition(NODE_LINKED, now_tick);
    } else {
        /* 井下/中继节点：上行有效则 LINKED，无效则 SCAN_LISTEN */
        if (uplink_valid) {
            s_ctx.active_link_is_uplink = LINK_UPLINK;
            _state_transition(NODE_LINKED, now_tick);
        } else {
            s_ctx.active_link_is_uplink = LINK_UPLINK;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
        }
    }
}

/**
 * @brief   更新幅度表（只记录最大幅值，不淘汰）
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
 * @brief   频对决选算法
 * @param[out] out_f0  选出的低频点
 * @param[out] out_f1  选出的高频点
 * @retval   0  成功选出频对
 * @retval  -1  有效频点数 < 2
 * @retval  -2  找不到满足间距要求的频对
 * @note    1. 筛选有效频点：amp > noise_floor + delta
 *          2. 找最强作为 f0
 *          3. 找次强且间距 ≥ 150 Hz 作为 f1
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
    
    *out_f0 = (uint16_t)SWEEP_FREQ_START + (uint16_t)f0_idx * SWEEP_FREQ_STEP;

    uint8_t  f1_idx = SWEEP_FREQ_INDEX_INVALID;
    uint16_t f1_amp_best = 0;
    
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
            f1_idx = cand;
        }
    }
    
    if (f1_idx == SWEEP_FREQ_INDEX_INVALID) {
        return -2;
    }

    *out_f1 = (uint16_t)SWEEP_FREQ_START + (uint16_t)f1_idx * SWEEP_FREQ_STEP;
    return 0;
}

/**
 * @brief   内部状态转移
 */
static void _state_transition(node_state_t next_state, uint32_t now_tick)
{
    if (s_ctx.state == next_state) {
        return;
    }

    s_ctx.state = next_state;
    s_ctx.state_enter_tick = now_tick;

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
 * @brief   判断总线是否静默超时
 */
static bool _is_bus_quiet(uint32_t now_tick)
{
    return (s_ctx.quiet_start_tick != QUIET_NOT_STARTED)
           && ((now_tick - s_ctx.quiet_start_tick) >= BUS_QUIET_TIMEOUT_MS);
}

/**
 * @brief   LINKED 状态下的断链检测
 * @note    【完全中心化扫频控制策略】
 *          所有断链情况由上位机诊断和控制，节点不自动扫频
 *          返回 LINK_OK，不触发自动 SCAN
 */
static link_check_result_t _check_link_failure_in_linked(uint32_t now_tick)
{
    (void)now_tick;
    return LINK_OK;
}

/* 状态进入函数 */

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
    
    if (s_ctx.sweep_start_tick == SWEEP_NOT_STARTED) {
        s_ctx.sweep_start_tick   = now_tick;
        s_ctx.sweep_timeout_tick = now_tick + SWEEP_SESSION_TIMEOUT_MS;
    }
    
    s_ctx.scan_freq_index     = 0;
    s_ctx.scan_freq_next_tick = now_tick + SCAN_FREQ_DURATION_MS;
    
    uint16_t first_freq = SWEEP_FREQ_START;
    (void)bsw_ad9833_sleep(false);
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, first_freq);
    (void)bsw_ad9833_select(BSW_AD9833_REG_0);
    
    bsw_led_set_state(LED_STATE_SCAN);
}

static void _state_entry_scan_wait_reply(uint32_t now_tick)
{
    (void)bsw_ad9833_sleep(true);
    bsw_adc_ringbuf_enable();
    
    memset(s_ctx.sweep_amp_table, 0, sizeof(s_ctx.sweep_amp_table));
    
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
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_start(MCAL_TIMER_TIM1);

    proto_rx_init(s_ctx.hard_id);

    node_freq_pair_t *active_freq = (s_ctx.active_link_is_uplink == LINK_UPLINK)
                                     ? &s_ctx.uplink_freq
                                     : &s_ctx.downlink_freq;

    /* 频点有效性校验 */
    if (active_freq->f0_hz == FREQ_NOT_LOCKED
        || active_freq->f1_hz == FREQ_NOT_LOCKED) {
        
        bool uplink_valid = (s_ctx.uplink_freq.f0_hz != FREQ_NOT_LOCKED 
                             && s_ctx.uplink_freq.f1_hz != FREQ_NOT_LOCKED);
        
        active_freq->f0_hz = FREQ_NOT_LOCKED;
        active_freq->f1_hz = FREQ_NOT_LOCKED;
        
        if (!app_node_fsm_has_uplink()) {
            /* 井上节点：保持 LINKED，等待上位机命令 */
            return;
        } else {
            if (!uplink_valid) {
                _state_transition(NODE_SCAN_LISTEN, now_tick);
            }
            return;
        }
    }

    bsw_bfsk_demod_start(active_freq->f0_hz,
                         active_freq->f1_hz,
                         PRE_BFSK_BIT_PERIOD_MS,
                         BFSK_DEMOD_TIMEOUT_MS,
                         s_ctx.hard_id);
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

/* PRE_LINKED 子状态机 */

typedef void (*phase_handler_t)(uint32_t now_tick);

static const phase_handler_t s_phase_handlers[] = {
    [PRE_QUIET_OBSERVE] = _phase_quiet_observe,
    [PRE_SELF_REPLY]    = _phase_self_reply,
    [PRE_WAIT_ACK]      = _phase_wait_ack,
};

/* PHASE1：等待自己的时间槽，期间监测总线避让 */
static void _phase_quiet_observe(uint32_t now_tick)
{
    if ((now_tick - s_ctx.pre_linked_last_check_tick) >= PRE_LINKED_CHECK_INTERVAL_MS) {
        s_ctx.pre_linked_last_check_tick = now_tick;
        uint32_t rms_sq = bsw_adc_ringbuf_calc_rms_sq();
        if (rms_sq > BSW_ADC_RINGBUF_SIGNAL_THRESHOLD) {
            /* 检测到别人应答，退让重新扫频 */
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

    /* 槽点到期，开始回发频对 */
    bsw_adc_ringbuf_disable();
    
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, s_ctx.uplink_freq.f0_hz);
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_1, s_ctx.uplink_freq.f1_hz);
    (void)bsw_ad9833_select(BSW_AD9833_REG_0);

    s_ctx.pre_linked_phase         = PRE_SELF_REPLY;
    s_ctx.pre_phase_enter_tick     = now_tick;
    s_ctx.self_reply_cur_is_f0     = true;
    s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
}

/* PHASE2：在 (f0, f1) 上交替发送 2s */
static void _phase_self_reply(uint32_t now_tick)
{
    if ((now_tick - s_ctx.pre_phase_enter_tick) >= PRE_REPLY_DURATION_MS) {
        /* 2s 发送完毕，进入 PHASE3 等待 ACK */
        (void)bsw_ad9833_sleep(true);
        
        bsw_bfsk_demod_start(s_ctx.uplink_freq.f0_hz,
                            s_ctx.uplink_freq.f1_hz,
                            PRE_BFSK_BIT_PERIOD_MS,
                            PRE_ACK_TIMEOUT_MS,
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

/* PHASE3：等待井上 ACK */
static void _phase_wait_ack(uint32_t now_tick)
{
    bsw_bfsk_demod_state_t demod_state = bsw_bfsk_demod_get_state();
    
    if (demod_state == BSW_BFSK_DEMOD_DONE_OK) {
        bsw_bfsk_demod_result_t demod_result;
        if (bsw_bfsk_demod_take_result(&demod_result) == BSW_BFSK_DEMOD_OK) {
            /* 初始化协议接收器（首次） */
            static bool proto_rx_initialized = false;
            if (!proto_rx_initialized) {
                proto_rx_init(bsw_node_id_get());
                proto_rx_initialized = true;
            }
            
            /* 把 bit 流喂给协议层 */
            proto_rx_feed_bits(demod_result.bit_buf, demod_result.bit_count);
            
            /* 检查是否有解析好的帧 */
            if (proto_rx_get_state() == PROTO_RX_DONE) {
                proto_frame_t frame;
                if (proto_rx_take_frame(&frame) == PROTO_ERR_OK) {
                    /* 检查是否是 ACK 帧（REPLY_FREQ_PAIR: type=0x3, msg_num=0x0）*/
                    if (frame.type_info.type == PROTO_TYPE_REPLY 
                        && frame.type_info.msg_num == PROTO_REPLY_FREQ_PAIR) {
                        if (frame.payload_len >= 4) {
                            uint16_t ack_f0 = frame.payload[0] | (frame.payload[1] << 8);
                            uint16_t ack_f1 = frame.payload[2] | (frame.payload[3] << 8);
                            
                            if (ack_f0 == s_ctx.uplink_freq.f0_hz && ack_f1 == s_ctx.uplink_freq.f1_hz) {
                                bsw_bfsk_demod_stop();
                                _state_transition(NODE_LINKED, now_tick);
                                return;
                            }
                        }
                    }
                }
            }
        }
        return;
    }
    else if (demod_state == BSW_BFSK_DEMOD_DONE_TIMEOUT) {
        bsw_bfsk_demod_stop();
        
        if (s_ctx.pre_ack_retry_count < PRE_ACK_MAX_RETRY) {
            s_ctx.pre_ack_retry_count++;
            
            (void)bsw_ad9833_sleep(false);
            (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, s_ctx.uplink_freq.f0_hz);
            (void)bsw_ad9833_set_freq(BSW_AD9833_REG_1, s_ctx.uplink_freq.f1_hz);
            (void)bsw_ad9833_select(BSW_AD9833_REG_0);
            
            s_ctx.pre_linked_phase         = PRE_SELF_REPLY;
            s_ctx.pre_phase_enter_tick     = now_tick;
            s_ctx.self_reply_cur_is_f0     = true;
            s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
        } else {
            s_ctx.uplink_freq.f0_hz  = FREQ_NOT_LOCKED;
            s_ctx.uplink_freq.f1_hz  = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f0_hz = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f1_hz = FREQ_NOT_LOCKED;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
        }
    }
}

/* 状态运行函数 */

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

/* ========== LINKED 状态：帧处理（委托给 app_frame_handler）========== */

/**
 * @brief   处理解调器接收到的 bit 流
 * @note    调用链：demod → proto → frame_handler
 *          1. 取 demod 结果（bit 流）
 *          2. 喂给 proto 状态机（bit → 字节 → 帧）
 *          3. 解析成功后分发给 app_frame_handler
 *          4. 设置标志位
 */
static void _handle_demod_result(void)
{
    bsw_bfsk_demod_state_t demod_state = bsw_bfsk_demod_get_state();
    
    if (demod_state == BSW_BFSK_DEMOD_DONE_OK) {
        bsw_bfsk_demod_result_t demod_result;
        if (bsw_bfsk_demod_take_result(&demod_result) == BSW_BFSK_DEMOD_OK) {
            /* 步骤 1: 初始化协议接收器（首次） */
            static bool proto_rx_initialized = false;
            if (!proto_rx_initialized) {
                proto_rx_init(bsw_node_id_get());
                proto_rx_initialized = true;
            }
            
            /* 步骤 2: 把 bit 流喂给协议层 */
            proto_err_t proto_err = proto_rx_feed_bits(demod_result.bit_buf, 
                                                       demod_result.bit_count);
            
            /* 步骤 3: 检查协议层是否有解析好的帧 */
            if (proto_rx_get_state() == PROTO_RX_DONE) {
                proto_frame_t frame;
                if (proto_rx_take_frame(&frame) == PROTO_ERR_OK) {
                    /* 步骤 4: 分发给业务层处理 */
                    app_frame_handler_dispatch(&frame);
                    
                    /* 步骤 5: 设置标志位（链路活跃）*/
                    app_node_fsm_flag_set_rx_up();
                }
            }
            
            /* 协议层错误处理 */
            if (proto_rx_get_state() == PROTO_RX_ERROR) {
                proto_err_t err = proto_rx_get_last_error();
                (void)err;  /* TODO: 错误统计 */
            }
        }
    }
    else if (demod_state == BSW_BFSK_DEMOD_DONE_TIMEOUT) {
        bsw_bfsk_demod_stop();
        /* 根据需要重新启动 */
    }
    else if (demod_state == BSW_BFSK_DEMOD_DONE_ERR) {
        /* 解调错误（缓冲区满等），复位并重新启动 */
        bsw_bfsk_demod_stop();
    }
}

static void _state_run_linked(uint32_t now_tick)
{
    /* 步骤 1: 处理解调结果（bit 流 → 帧 → 业务） */
    _handle_demod_result();
    
    /* ========== 断链检测（上行链路） ========== */
    
    /* 场景 2：所有标志位为 0（链路完全断裂，频点可能已变化）
     * - 收不到上级信号
     * - 也无下行通信活动
     * → 进入 SCAN_LISTEN 被动等待上级重新扫频
     * 注意：井上节点（无上级）不会触发此场景，因为它的 rx_up 和 fwd_up 永远为 1 */
    if ((s_ctx.flags.flag_rx_up_ok  == 0)
        && (s_ctx.flags.flag_fwd_dn_ok == 0)
        && (s_ctx.flags.flag_rx_dn_ok  == 0)
        && (s_ctx.flags.flag_fwd_up_ok == 0))
    {
        if (_is_bus_quiet(now_tick)) {
            /* 井上节点不会进入这里（rx_up 永远为 1）
             * 井下和中继节点：完全孤立，进入监听 */
            _state_transition(NODE_SCAN_LISTEN, now_tick);
            return;
        }
    }
    
    /* 场景 3：有下行通信活动，但向上转发失败（上行链路中途断裂）
     * - 下级节点正常工作（能接收或转发给下级）
     * - 但无法转发给上级（上行频点可能变化）
     * → 进入 SCAN_LISTEN 重新监听上级新频点
     * 注意：井上节点（无上级）和井下节点（无下级）不会触发此场景 */
    if (app_node_fsm_has_uplink() && app_node_fsm_has_downlink()) {
        /* 仅中继节点检测此场景 */
        if (((s_ctx.flags.flag_fwd_dn_ok != 0) || (s_ctx.flags.flag_rx_dn_ok != 0))
            && (s_ctx.flags.flag_fwd_up_ok == 0))
        {
            if (_is_bus_quiet(now_tick)) {
                _state_transition(NODE_SCAN_LISTEN, now_tick);
                return;
            }
        }
    }
    
    /* ========== 断链检测（下行链路） ========== */
    
    /* 场景一：上行通畅但下行异常 → 进入 SCAN 主动扫频重建下行 */
    if (_check_link_failure_in_linked(now_tick) == LINK_FAIL_NEED_SCAN) {
        _state_transition(NODE_SCAN, now_tick);
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
    
    /* 初始化频点为未锁定状态（Flash 读取失败时的默认值） */
    s_ctx.uplink_freq.f0_hz   = FREQ_NOT_LOCKED;
    s_ctx.uplink_freq.f1_hz   = FREQ_NOT_LOCKED;
    s_ctx.downlink_freq.f0_hz = FREQ_NOT_LOCKED;
    s_ctx.downlink_freq.f1_hz = FREQ_NOT_LOCKED;

    /* 初始化活动链路为未激活 */
    s_ctx.active_link_is_uplink = LINK_NONE;

    bsw_node_id_init(hard_id);
    bsw_led_init();  /* 初始化 LED 指示灯 */
    bsw_version_init();  /* 初始化版本模块 */

    /* 初始化 Flash 频点存储模块 */
    bsw_freq_storage_init();

    /* 一次性读取上行 + 下行两对频点
     * - Flash 有备份 → 直接使用（快速恢复）
     * - Flash 无备份 → load_all 内部把所有输出置 0（即 FREQ_NOT_LOCKED）
     *
     * 注意：这里不直接进入 LINKED 状态，因为：
     * 1. 对端可能已经掉电重启，频点可能已变化
     * 2. 需要先通过扫频/监听重新建立通信
     * 3. 只是将频点作为"上次锁定频点"供诊断和优化使用
     *
     * 进入 LINKED 时会在 _state_entry_linked() 中校验频点有效性：
     * - 有效 → 正常进入 LINKED
     * - 无效 → 跳转到 SCAN/SCAN_LISTEN（双保险）
     */
    uint16_t saved_uf0 = 0, saved_uf1 = 0, saved_df0 = 0, saved_df1 = 0;
    (void)bsw_freq_storage_load_all(&saved_uf0, &saved_uf1, &saved_df0, &saved_df1);
    /* load_all 内部已做合理性校验（范围、非零、非 0xFFFF），失败时输出为 0 */
    
    /* APP 层二次校验：精确匹配频点表（防止 Flash 垃圾数据）
     * 
     * BSW 层只做范围检查（100~2000 Hz），这里做精确表内校验
     * 只有频点对的两个频点都在表内才认为有效，否则清零
     */
    bool uf0_in_table = (saved_uf0 != 0) && _is_freq_in_table(saved_uf0);
    bool uf1_in_table = (saved_uf1 != 0) && _is_freq_in_table(saved_uf1);
    bool df0_in_table = (saved_df0 != 0) && _is_freq_in_table(saved_df0);
    bool df1_in_table = (saved_df1 != 0) && _is_freq_in_table(saved_df1);
    
    if (uf0_in_table && uf1_in_table) {
        s_ctx.uplink_freq.f0_hz = saved_uf0;
        s_ctx.uplink_freq.f1_hz = saved_uf1;
    } else {
        s_ctx.uplink_freq.f0_hz = FREQ_NOT_LOCKED;
        s_ctx.uplink_freq.f1_hz = FREQ_NOT_LOCKED;
    }
    
    if (df0_in_table && df1_in_table) {
        s_ctx.downlink_freq.f0_hz = saved_df0;
        s_ctx.downlink_freq.f1_hz = saved_df1;
    } else {
        s_ctx.downlink_freq.f0_hz = FREQ_NOT_LOCKED;
        s_ctx.downlink_freq.f1_hz = FREQ_NOT_LOCKED;
    }

    /* 根据 Flash 备份的频点情况选择初始状态 */
    _select_initial_state();
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
                
                /* 备份频对到 Flash（扫频方刚确认下行频点，保留上行原值） */
                int32_t ret = bsw_freq_storage_save_all(
                    s_ctx.uplink_freq.f0_hz, s_ctx.uplink_freq.f1_hz,
                    f0, f1);
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

    /* 备份频对到 Flash（被扫方刚确认上行频点，保留下行原值） */
    int32_t ret = bsw_freq_storage_save_all(
        f0, f1,
        s_ctx.downlink_freq.f0_hz, s_ctx.downlink_freq.f1_hz);
    (void)ret;  /* 备份失败不影响进入 LINKED，只是下次掉电无法快速恢复 */

    _state_transition(NODE_LINKED, HAL_GetTick());
}

/* ========== 事务标志位操作 ========== */

void app_node_fsm_flag_set_rx_up(void)
{
    s_ctx.flags.flag_rx_up_ok = FLAG_SET;
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

void app_node_fsm_flag_set_fwd_dn(void)
{
    s_ctx.flags.flag_fwd_dn_ok = FLAG_SET;
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

void app_node_fsm_flag_set_rx_dn(void)
{
    s_ctx.flags.flag_rx_dn_ok = FLAG_SET;
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

void app_node_fsm_flag_set_fwd_up(void)
{
    s_ctx.flags.flag_fwd_up_ok = FLAG_SET;
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

void app_node_fsm_flag_clear_rx_up(void)
{
    s_ctx.flags.flag_rx_up_ok = FLAG_CLEARED;
}

void app_node_fsm_flag_clear_fwd_dn(void)
{
    s_ctx.flags.flag_fwd_dn_ok = FLAG_CLEARED;
}

void app_node_fsm_flag_clear_rx_dn(void)
{
    s_ctx.flags.flag_rx_dn_ok = FLAG_CLEARED;
}

void app_node_fsm_flag_clear_fwd_up(void)
{
    s_ctx.flags.flag_fwd_up_ok = FLAG_CLEARED;
}

void app_node_fsm_on_scan_order(uint8_t scan_direction)
{
    uint32_t now_tick = HAL_GetTick();
    
    /* 【中心化扫频控制】
     * - 井上通过 CMD_SCAN_ORDER 指令指定节点扫频
     * - scan_direction: 0=扫上行（一般不用），1=扫下行
     */
    
    if (scan_direction == 1) {
        /* 扫下行：设置活动链路为下行，进入 SCAN 状态 */
        if (app_node_fsm_has_downlink()) {
            s_ctx.active_link_is_uplink = LINK_DOWNLINK;
            _state_transition(NODE_SCAN, now_tick);
        }
        /* 井下节点没有下级，忽略此命令 */
    } else {
        /* 扫上行：一般不需要，因为上行断了应该自动进入 SCAN_LISTEN
         * 但为了协议完整性，也支持这个方向 */
        if (app_node_fsm_has_uplink()) {
            s_ctx.active_link_is_uplink = LINK_UPLINK;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
        }
        /* 井上节点没有上级，忽略此命令 */
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

/* ========== 节点角色判断 ========== */

node_role_t app_node_fsm_get_role(void)
{
    if (s_ctx.hard_id == BSW_NODE_ADDR_GROUND) {
        return NODE_ROLE_SURFACE;  /* 井上节点 */
    } else if (s_ctx.hard_id == BSW_NODE_ADDR_UNDERGROUND) {
        return NODE_ROLE_UNDERGROUND;  /* 井下节点 */
    } else {
        return NODE_ROLE_RELAY;  /* 中继节点 */
    }
}

bool app_node_fsm_has_uplink(void)
{
    /* 井上节点（BSW_NODE_ADDR_GROUND = 0）无上级，其余都有上级 */
    return (s_ctx.hard_id != BSW_NODE_ADDR_GROUND);
}

bool app_node_fsm_has_downlink(void)
{
    /* 井下节点（BSW_NODE_ADDR_UNDERGROUND = 0xF）无下级，其余都有下级 */
    return (s_ctx.hard_id != BSW_NODE_ADDR_UNDERGROUND);
}
