/**
 * @file    app_node_fsm.c
 * @brief   井下节点状态机实现
 * @details
 *  - 状态机状态转换：BOOT → SCAN_LISTEN ↔ SCAN → PRE_LINKED → LINKED
 *  - 支持三种节点角色：井上、井下、中继
 *  - 双频 BFSK 通信协议实现
 */

// ============================================================================
// 状态机配置宏定义
// ============================================================================

// 解调超时（ms）
#define APP_NODE_FSM_DEMOD_TIMEOUT_MS                 (5000U)

// 扫频幅值有效阈值
#define APP_NODE_FSM_SWEEP_AMP_VALID_THRESHOLD        (0U)

// 扫频未启动标记
#define APP_NODE_FSM_SWEEP_NOT_STARTED                (0U)

// 预链接检查周期（ms）
#define APP_NODE_FSM_PRE_LINKED_CHECK_INTERVAL_MS     (40U)

// 槽点永不到达（无效值）
#define APP_NODE_FSM_SLOT_NEVER_ARRIVE                (0xFFFFFFFFU)

// 链路无效状态
#define APP_NODE_FSM_LINK_NONE                        (0U)

// ============================================================================
// 头文件包含
// ============================================================================

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
#include "app_frame_handler.h"

// ============================================================================
// 内部类型定义
// ============================================================================

/**
 * @brief 链路检测结果
 * @details 用于判断是否需要扫频或监听
 */
typedef enum {
    LINK_OK               = 0,  // 链路正常
    LINK_FAIL_NEED_SCAN   = 1,  // 链路断裂，需要主动扫频
    LINK_FAIL_NEED_LISTEN = 2,  // 链路断裂，需要被动监听
} LinkCheckResult_t;

// ============================================================================
// 全局上下文
// ============================================================================

/**
 * @brief 状态机全局上下文
 * @rationale: 状态机需要在多个状态间持久化数据，必须是静态全局变量
 * @thread-safe: 主任务独占访问，ISR 只读状态字段
 */
static app_node_ctx_t s_ctx;

// ============================================================================
// 前向声明 - 状态转移函数
// ============================================================================

/**
 * @brief 状态转移处理
 * @param next_state 目标状态
 * @param now_tick 当前时刻（ms）
 * @details 自动调用对应状态的进入函数
 */
static void _state_transition(node_state_t next_state, uint32_t now_tick);

// ============================================================================
// 前向声明 - 状态进入函数
// ============================================================================

static void _state_entry_boot(uint32_t now_tick);
static void _state_entry_scan_listen(uint32_t now_tick);
static void _state_entry_scan(uint32_t now_tick);
static void _state_entry_scan_wait_reply(uint32_t now_tick);
static void _state_entry_pre_linked(uint32_t now_tick);
static void _state_entry_linked(uint32_t now_tick);
static void _state_entry_sleep(uint32_t now_tick);
static void _state_entry_fault(uint32_t now_tick);

// ============================================================================
// 前向声明 - 状态运行函数
// ============================================================================

static void _state_run_boot(uint32_t now_tick);
static void _state_run_scan_listen(uint32_t now_tick);
static void _state_run_scan(uint32_t now_tick);
static void _state_run_scan_wait_reply(uint32_t now_tick);
static void _state_run_pre_linked(uint32_t now_tick);
static void _state_run_linked(uint32_t now_tick);
static void _state_run_sleep(uint32_t now_tick);
static void _state_run_fault(uint32_t now_tick);

// ============================================================================
// 前向声明 - 预链接子状态机处理函数
// ============================================================================

static void _phase_quiet_observe(uint32_t now_tick);
static void _phase_self_reply(uint32_t now_tick);
static void _phase_wait_ack(uint32_t now_tick);

/**
 * @brief 频率 → 扫频表索引映射
 * @param f_hz 频率值（Hz）
 * @return 0~18 表示合法索引；SWEEP_FREQ_INDEX_INVALID 表示频率超出容差
 * 
 * @details
 *  19 个频点：125, 175, 225, ..., 1025 Hz
 *  步进：50 Hz
 *  容差：±15 Hz
 *  
 *  算法：
 *  1. 检查下界（SWEEP_FREQ_START - TOLERANCE）
 *  2. 计算相对偏移和最近的表内频点
 *  3. 检查上界（SWEEP_FREQ_START + TOLERANCE）
 *  4. 验证偏差在容差范围内
 */
static uint8_t _sweep_freq_to_index(uint16_t f_hz)
{
    // 下界检查
    if (f_hz < (uint16_t)(SWEEP_FREQ_START - SWEEP_FREQ_TOLERANCE_HZ))
    {
        return SWEEP_FREQ_INDEX_INVALID;
    }

    // 计算相对起始频点的偏移
    int32_t delta = (int32_t)f_hz - (int32_t)SWEEP_FREQ_START;
    if (delta < 0)
    {
        delta = 0;
    }

    // 计算最近的表内频点索引
    int32_t idx    = delta / (int32_t)SWEEP_FREQ_STEP;
    int32_t offset = delta % (int32_t)SWEEP_FREQ_STEP;

    // 四舍五入：如果余数 > 步进的一半，则进到下一个频点
    if (offset > (int32_t)(SWEEP_FREQ_STEP / 2))
    {
        ++idx;
        offset = (int32_t)SWEEP_FREQ_STEP - offset;
    }

    // 索引范围检查
    if (idx < 0 || idx >= (int32_t)SWEEP_FREQ_COUNT)
    {
        return SWEEP_FREQ_INDEX_INVALID;
    }

    // 容差范围检查
    if (offset > (int32_t)SWEEP_FREQ_TOLERANCE_HZ)
    {
        return SWEEP_FREQ_INDEX_INVALID;
    }

    return (uint8_t)idx;
}

/**
 * @brief 校验频点是否在有效频点表内
 * @param f_hz 待校验的频点（Hz）
 * @return true = 在表内且容差范围内，false = 不在表内
 */
static bool _is_freq_in_table(uint16_t f_hz)
{
    return (_sweep_freq_to_index(f_hz) != SWEEP_FREQ_INDEX_INVALID);
}

/**
 * @brief 根据频点有效性和节点角色选择初始状态
 * 
 * @details
 *  决策逻辑：
 *  - 井上节点（无上级）→ LINKED（串口直连上位机，永远可控）
 *  - 井下/中继节点：
 *    * 上行频点有效 → LINKED（恢复链路）
 *    * 上行频点无效 → SCAN_LISTEN（失联，被动监听上级扫频）
 */
static void _select_initial_state(void)
{
    uint32_t now_tick = HAL_GetTick();
    s_ctx.state_enter_tick = now_tick;

    // 检查上行频点有效性（两个频点都非 FREQ_NOT_LOCKED）
    bool uplink_valid = (s_ctx.uplink_freq.f0_hz != FREQ_NOT_LOCKED 
                         && s_ctx.uplink_freq.f1_hz != FREQ_NOT_LOCKED);

    if (!app_node_fsm_has_uplink())
    {
        // 井上节点：无上级，串口直连上位机
        s_ctx.active_link_is_uplink = LINK_DOWNLINK;
        _state_transition(NODE_LINKED, now_tick);
    }
    else
    {
        // 井下/中继节点：根据上行频点有效性决定初始状态
        if (uplink_valid)
        {
            s_ctx.active_link_is_uplink = LINK_UPLINK;
            _state_transition(NODE_LINKED, now_tick);
        }
        else
        {
            s_ctx.active_link_is_uplink = LINK_UPLINK;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
        }
    }
}

/**
 * @brief 更新幅度表（保留各频点的最大幅值）
 * @param idx 频点索引（0~18）
 * @param amplitude 当前检测到的幅度
 * 
 * @details
 *  策略：只记录最大值，不淘汰
 *  用于扫频过程中累积各频点的信号强度
 */
static void _update_amp_table(uint8_t idx, uint16_t amplitude)
{
    // 索引范围检查
    if (idx >= SWEEP_FREQ_COUNT)
    {
        return;
    }
    
    // 只更新更大的幅值
    if (amplitude > s_ctx.sweep_amp_table[idx])
    {
        s_ctx.sweep_amp_table[idx] = amplitude;
    }
}

/**
 * @brief 频对决选算法
 * @param[out] p_out_f0 选出的低频点（输出）
 * @param[out] p_out_f1 选出的高频点（输出）
 * @return 0 = 成功选出频对，-1 = 有效频点不足，-2 = 找不到满足间距的频对
 * 
 * @details
 *  算法步骤：
 *  1. 找出幅值最大的频点
 *  2. 计算强度门限 = max_amp * 70%
 *  3. 筛选超过门限的频点
 *  4. 在强频点中找幅值最接近的两个，且满足最小间隔要求（≥150Hz）
 *  
 *  决策依据：
 *  - 幅值相近的频点更可能是同一对信号源
 *  - 最小间隔保证频点可区分
 */
static int32_t _select_freq_pair(uint16_t *p_out_f0, uint16_t *p_out_f1)
{
    // 参数检查
    if (p_out_f0 == NULL || p_out_f1 == NULL)
    {
        return -1;
    }

    // 步骤 1：找到幅值最大的频点
    uint16_t max_amp = 0;
    
    for (uint8_t i = 0; i < SWEEP_FREQ_COUNT; ++i)
    {
        if (s_ctx.sweep_amp_table[i] > max_amp)
        {
            max_amp = s_ctx.sweep_amp_table[i];
        }
    }
    
    if (max_amp == 0)
    {
        return -1;  // 无有效信号
    }
    
    // 步骤 2：计算强度门限 = max_amp * 70%
    uint16_t strength_threshold = (max_amp * SWEEP_AMP_STRENGTH_THRESHOLD_PERCENT) / 100u;
    
    // 步骤 3：筛选超过门限的频点
    uint8_t strong_idx[SWEEP_FREQ_COUNT];
    uint8_t strong_cnt = 0;
    
    for (uint8_t i = 0; i < SWEEP_FREQ_COUNT; ++i)
    {
        if (s_ctx.sweep_amp_table[i] >= strength_threshold)
        {
            strong_idx[strong_cnt++] = i;
        }
    }
    
    if (strong_cnt < SWEEP_MIN_VALID_FREQ_CNT)
    {
        return -1;  // 强度足够的频点不足
    }
    
    // 步骤 4：在强频点中找幅值最接近的两个，且满足最小间隔
    uint8_t  best_f0_idx = SWEEP_FREQ_INDEX_INVALID;
    uint8_t  best_f1_idx = SWEEP_FREQ_INDEX_INVALID;
    uint16_t best_amp_diff = 0xFFFFU;  // 幅值差异最小为目标
    
    for (uint8_t i = 0; i < strong_cnt; ++i)
    {
        for (uint8_t j = i + 1; j < strong_cnt; ++j)
        {
            uint8_t  idx_a = strong_idx[i];
            uint8_t  idx_b = strong_idx[j];
            uint16_t amp_a = s_ctx.sweep_amp_table[idx_a];
            uint16_t amp_b = s_ctx.sweep_amp_table[idx_b];
            
            // 检查频率间隔是否满足最小要求
            uint8_t  gap_idx = (idx_b > idx_a) ? (idx_b - idx_a) : (idx_a - idx_b);
            uint16_t gap_hz  = (uint16_t)gap_idx * SWEEP_FREQ_STEP;
            
            if (gap_hz < SWEEP_MIN_FREQ_GAP_HZ)
            {
                continue;  // 间隔不足，跳过
            }
            
            // 计算幅值差异
            uint16_t amp_diff = (amp_a > amp_b) ? (amp_a - amp_b) : (amp_b - amp_a);
            
            // 选择差异最小的那一对
            if (amp_diff < best_amp_diff)
            {
                best_amp_diff = amp_diff;
                
                // f0 为幅值较大的，f1 为幅值较小的
                if (amp_a >= amp_b)
                {
                    best_f0_idx = idx_a;
                    best_f1_idx = idx_b;
                }
                else
                {
                    best_f0_idx = idx_b;
                    best_f1_idx = idx_a;
                }
            }
        }
    }
    
    if (best_f0_idx == SWEEP_FREQ_INDEX_INVALID)
    {
        return -2;  // 找不到满足间隔要求的频点对
    }

    // 输出选中的频点
    *p_out_f0 = (uint16_t)SWEEP_FREQ_START + (uint16_t)best_f0_idx * SWEEP_FREQ_STEP;
    *p_out_f1 = (uint16_t)SWEEP_FREQ_START + (uint16_t)best_f1_idx * SWEEP_FREQ_STEP;
    return 0;
}

/**
 * @brief 状态转移处理
 * @param next_state 目标状态
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  使用函数表模式实现状态转移：
 *  1. 检查是否需要转移（避免重复进入）
 *  2. 更新状态和进入时刻
 *  3. 查表调用对应的状态进入函数
 */
static void _state_transition(node_state_t next_state, uint32_t now_tick)
{
    // 避免重复进入同一状态
    if (s_ctx.state == next_state)
    {
        return;
    }

    // 更新状态
    s_ctx.state = next_state;
    s_ctx.state_enter_tick = now_tick;

    // 状态进入函数表
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

    // 调用对应的进入函数
    if (next_state < sizeof(s_entry_table) / sizeof(s_entry_table[0]))
    {
        state_entry_t entry_fn = s_entry_table[next_state];
        if (entry_fn != NULL)
        {
            entry_fn(now_tick);
        }
    }
}

/**
 * @brief 判断总线是否静默超时
 * @param now_tick 当前时刻（ms）
 * @return true = 总线静默超时，false = 未超时或静默未开始
 * 
 * @details
 *  用于检测通信链路是否断裂：
 *  - 静默开始后超过 BUS_QUIET_TIMEOUT_MS 未收到任何信号
 */
static bool _is_bus_quiet(uint32_t now_tick)
{
    return (s_ctx.quiet_start_tick != QUIET_NOT_STARTED)
           && ((now_tick - s_ctx.quiet_start_tick) >= BUS_QUIET_TIMEOUT_MS);
}

/**
 * @brief LINKED 状态下的断链检测
 * @param now_tick 当前时刻（ms）
 * @return 链路检测结果
 * 
 * @details
 *  【完全中心化扫频控制策略】
 *  所有断链情况由上位机诊断和控制，节点不自动扫频
 *  始终返回 LINK_OK，不触发自动 SCAN
 */
static LinkCheckResult_t _check_link_failure_in_linked(uint32_t now_tick)
{
    (void)now_tick;
    return LINK_OK;
}

// ============================================================================
// 状态进入函数实现
// ============================================================================

/**
 * @brief BOOT 状态进入函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  启动状态：关闭所有外设，等待初始化完成
 *  - 禁用 ADC 环形缓冲
 *  - 停止定时器输入捕获
 *  - 关闭所有 LED 指示灯
 */
static void _state_entry_boot(uint32_t now_tick)
{
    (void)now_tick;
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    bsw_led_set_state(LED_STATE_ALL_OFF);
}

/**
 * @brief SCAN_LISTEN 状态进入函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  被动监听状态：等待上级扫频信号
 *  - 启用 ADC 环形缓冲（接收信号）
 *  - 停止定时器输入捕获（不解调）
 *  - 清空幅度表
 *  - 重置扫频计时器
 *  - LED 显示监听状态
 */
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

/**
 * @brief SCAN 状态进入函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  主动扫频状态：向下级发送 19 个频点
 *  - 禁用 ADC 环形缓冲（发送模式）
 *  - 停止定时器输入捕获
 *  - 启动扫频会话计时器
 *  - 初始化扫频索引
 *  - 唤醒 AD9833 并发送第一个频点
 *  - LED 显示扫频状态
 */
static void _state_entry_scan(uint32_t now_tick)
{
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    
    // 启动扫频会话计时器（首次扫频）
    if (s_ctx.sweep_start_tick == APP_NODE_FSM_SWEEP_NOT_STARTED)
    {
        s_ctx.sweep_start_tick   = now_tick;
        s_ctx.sweep_timeout_tick = now_tick + SWEEP_SESSION_TIMEOUT_MS;
    }
    
    // 初始化扫频参数
    s_ctx.scan_freq_index     = 0;
    s_ctx.scan_freq_next_tick = now_tick + SCAN_FREQ_DURATION_MS;
    
    // 发送第一个频点
    uint16_t first_freq = SWEEP_FREQ_START;
    (void)bsw_ad9833_sleep(false);
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, first_freq);
    (void)bsw_ad9833_select(BSW_AD9833_REG_0);
    
    bsw_led_set_state(LED_STATE_SCAN);
}

/**
 * @brief SCAN_WAIT_REPLY 状态进入函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  等待下级回应状态：监听下级的 1010 交替波形
 *  - AD9833 进入休眠（停止发送）
 *  - 启用 ADC 环形缓冲（接收模式）
 *  - 清空幅度表
 *  - 初始化回应检测参数
 *  - LED 显示监听状态
 */
static void _state_entry_scan_wait_reply(uint32_t now_tick)
{
    // 停止发送，进入接收模式
    (void)bsw_ad9833_sleep(true);
    bsw_adc_ringbuf_enable();
    
    // 清空幅度表
    memset(s_ctx.sweep_amp_table, 0, sizeof(s_ctx.sweep_amp_table));
    
    // 初始化回应检测参数
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

/**
 * @brief PRE_LINKED 状态进入函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  预链接状态：等待时间槽并回发频对
 *  - 启用 ADC 环形缓冲（监测总线）
 *  - 停止定时器输入捕获
 *  - 初始化子状态机参数
 *  - LED 显示预链接状态
 */
static void _state_entry_pre_linked(uint32_t now_tick)
{
    bsw_adc_ringbuf_enable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    
    // 初始化预链接子状态机
    s_ctx.pre_linked_last_check_tick = now_tick;
    s_ctx.pre_linked_phase           = PRE_QUIET_OBSERVE;
    s_ctx.pre_phase_enter_tick       = now_tick;
    s_ctx.self_reply_cur_is_f0       = true;
    s_ctx.pre_ack_retry_count        = 0;
    
    bsw_led_set_state(LED_STATE_PRELINK);
}

/**
 * @brief LINKED 状态进入函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  链接状态：正常通信模式
 *  - 禁用 ADC 环形缓冲（使用 BFSK 解调）
 *  - 启动定时器输入捕获（BFSK 解调）
 *  - 初始化协议接收器
 *  - 启动 BFSK 解调器
 *  - LED 显示链接状态
 *  
 *  频点有效性校验：
 *  - 无效频点 → 跳转到 SCAN_LISTEN（双保险）
 *  - 井上节点无频点时保持 LINKED（等待上位机命令）
 */
static void _state_entry_linked(uint32_t now_tick)
{
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_start(MCAL_TIMER_TIM1);

    proto_rx_init(s_ctx.hard_id);

    // 选择活动链路频点
    node_freq_pair_t *p_active_freq = (s_ctx.active_link_is_uplink == LINK_UPLINK)
                                       ? &s_ctx.uplink_freq
                                       : &s_ctx.downlink_freq;

    // 频点有效性校验
    if (p_active_freq->f0_hz == FREQ_NOT_LOCKED
        || p_active_freq->f1_hz == FREQ_NOT_LOCKED)
    {
        bool uplink_valid = (s_ctx.uplink_freq.f0_hz != FREQ_NOT_LOCKED 
                             && s_ctx.uplink_freq.f1_hz != FREQ_NOT_LOCKED);
        
        p_active_freq->f0_hz = FREQ_NOT_LOCKED;
        p_active_freq->f1_hz = FREQ_NOT_LOCKED;
        
        if (!app_node_fsm_has_uplink())
        {
            // 井上节点：保持 LINKED，等待上位机命令
            return;
        }
        else
        {
            if (!uplink_valid)
            {
                _state_transition(NODE_SCAN_LISTEN, now_tick);
            }
            return;
        }
    }

    // 启动 BFSK 解调器
    bsw_bfsk_demod_start(p_active_freq->f0_hz,
                         p_active_freq->f1_hz,
                         PRE_BFSK_BIT_PERIOD_MS,
                         APP_NODE_FSM_DEMOD_TIMEOUT_MS,
                         s_ctx.hard_id);
    bsw_led_set_state(LED_STATE_LINKED);
}

/**
 * @brief SLEEP 状态进入函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  休眠状态：总线长时间无活动
 *  - 禁用所有外设
 *  - 关闭 LED
 */
static void _state_entry_sleep(uint32_t now_tick)
{
    (void)now_tick;
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    bsw_led_set_state(LED_STATE_ALL_OFF);
}

/**
 * @brief FAULT 状态进入函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  故障状态：系统异常
 *  - 禁用所有外设
 *  - LED 显示故障状态
 */
static void _state_entry_fault(uint32_t now_tick)
{
    (void)now_tick;
    bsw_adc_ringbuf_disable();
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    bsw_led_set_state(LED_STATE_FAULT);
}

// ============================================================================
// PRE_LINKED 子状态机实现
// ============================================================================

/**
 * @brief 预链接子状态处理函数表
 * @details 使用函数表模式，避免 switch 语句
 */
typedef void (*phase_handler_t)(uint32_t now_tick);

static const phase_handler_t s_phase_handlers[] = {
    [PRE_QUIET_OBSERVE] = _phase_quiet_observe,
    [PRE_SELF_REPLY]    = _phase_self_reply,
    [PRE_WAIT_ACK]      = _phase_wait_ack,
};

/**
 * @brief PHASE1：等待自己的时间槽，期间监测总线避让
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  等待逻辑：
 *  1. 周期性检测总线信号（PRE_LINKED_CHECK_INTERVAL_MS）
 *  2. 检测到他人应答 → 退让，回到 SCAN_LISTEN
 *  3. 到达自己的时间槽 → 进入 PHASE2 回发频对
 */
static void _phase_quiet_observe(uint32_t now_tick)
{
    // 周期性检测总线信号
    if ((now_tick - s_ctx.pre_linked_last_check_tick) >= APP_NODE_FSM_PRE_LINKED_CHECK_INTERVAL_MS)
    {
        s_ctx.pre_linked_last_check_tick = now_tick;
        
        uint32_t rms_sq = bsw_adc_ringbuf_calc_rms_sq();
        if (rms_sq > BSW_ADC_RINGBUF_SIGNAL_THRESHOLD)
        {
            // 检测到别人应答，退让重新扫频
            s_ctx.uplink_freq.f0_hz  = FREQ_NOT_LOCKED;
            s_ctx.uplink_freq.f1_hz  = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f0_hz = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f1_hz = FREQ_NOT_LOCKED;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
            return;
        }
    }

    // 计算自己的时间槽
    uint32_t my_slot_offset = bsw_node_id_get_reply_delay_ms();
    if (my_slot_offset == BSW_NODE_ID_SLOT_INVALID)
    {
        my_slot_offset = PRE_QUIET_OBSERVE_MAX_MS;
    }

    // 检查是否到达时间槽
    uint32_t since_state_enter = (uint32_t)(now_tick - s_ctx.state_enter_tick);
    if (since_state_enter < my_slot_offset
        && since_state_enter < PRE_QUIET_OBSERVE_MAX_MS)
    {
        return;
    }

    // 槽点到期，开始回发频对
    bsw_adc_ringbuf_disable();
    
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, s_ctx.uplink_freq.f0_hz);
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_1, s_ctx.uplink_freq.f1_hz);
    (void)bsw_ad9833_select(BSW_AD9833_REG_0);

    s_ctx.pre_linked_phase         = PRE_SELF_REPLY;
    s_ctx.pre_phase_enter_tick     = now_tick;
    s_ctx.self_reply_cur_is_f0     = true;
    s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
}

/**
 * @brief PHASE2：在 (f0, f1) 上交替发送 2s
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  发送 1010 交替波形：
 *  - 每个码元周期切换一次频率
 *  - 持续 PRE_REPLY_DURATION_MS (2000ms)
 *  - 完成后进入 PHASE3 等待 ACK
 */
static void _phase_self_reply(uint32_t now_tick)
{
    // 检查是否发送完成
    if ((now_tick - s_ctx.pre_phase_enter_tick) >= PRE_REPLY_DURATION_MS)
    {
        // 2s 发送完毕，进入 PHASE3 等待 ACK
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
    
    // 检查是否到达下一个码元切换时刻
    if ((int32_t)(now_tick - s_ctx.self_reply_next_bit_tick) < 0)
    {
        return;
    }
    
    // 切换频率
    if (s_ctx.self_reply_cur_is_f0)
    {
        (void)bsw_ad9833_select(BSW_AD9833_REG_1);
        s_ctx.self_reply_cur_is_f0 = false;
    }
    else
    {
        (void)bsw_ad9833_select(BSW_AD9833_REG_0);
        s_ctx.self_reply_cur_is_f0 = true;
    }
    s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
}

/**
 * @brief PHASE3：等待井上节点的 ACK 确认
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  ACK 接收逻辑：
 *  1. 解调器接收到数据 → 解析帧 → 检查是否是频点 ACK
 *  2. ACK 匹配 → 进入 LINKED 状态
 *  3. 超时 → 重发频对（最多 PRE_ACK_MAX_RETRY 次）
 *  4. 重发次数用尽 → 回到 SCAN_LISTEN
 */
static void _phase_wait_ack(uint32_t now_tick)
{
    bsw_bfsk_demod_state_t demod_state = bsw_bfsk_demod_get_state();
    
    if (demod_state == BSW_BFSK_DEMOD_DONE_OK)
    {
        bsw_bfsk_demod_result_t demod_result;
        if (bsw_bfsk_demod_take_result(&demod_result) == BSW_BFSK_DEMOD_OK)
        {
            // 初始化协议接收器（首次）
            static bool s_proto_rx_initialized = false;
            if (!s_proto_rx_initialized)
            {
                proto_rx_init(bsw_node_id_get());
                s_proto_rx_initialized = true;
            }
            
            // 把 bit 流喂给协议层
            proto_rx_feed_bits(demod_result.bit_buf, demod_result.bit_count);
            
            // 检查是否有解析好的帧
            if (proto_rx_get_state() == PROTO_RX_DONE)
            {
                proto_frame_t frame;
                if (proto_rx_take_frame(&frame) == PROTO_ERR_OK)
                {
                    // 检查是否是 ACK 帧（REPLY_FREQ_PAIR: type=0x3, msg_num=0x0）
                    if (frame.type_info.type == PROTO_TYPE_REPLY 
                        && frame.type_info.msg_num == PROTO_REPLY_FREQ_PAIR)
                    {
                        if (frame.payload_len >= 4)
                        {
                            uint16_t ack_f0 = frame.payload[0] | (frame.payload[1] << 8);
                            uint16_t ack_f1 = frame.payload[2] | (frame.payload[3] << 8);
                            
                            if (ack_f0 == s_ctx.uplink_freq.f0_hz && ack_f1 == s_ctx.uplink_freq.f1_hz)
                            {
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
    else if (demod_state == BSW_BFSK_DEMOD_DONE_TIMEOUT)
    {
        bsw_bfsk_demod_stop();
        
        if (s_ctx.pre_ack_retry_count < PRE_ACK_MAX_RETRY)
        {
            s_ctx.pre_ack_retry_count++;
            
            // 重新发送频对
            (void)bsw_ad9833_sleep(false);
            (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, s_ctx.uplink_freq.f0_hz);
            (void)bsw_ad9833_set_freq(BSW_AD9833_REG_1, s_ctx.uplink_freq.f1_hz);
            (void)bsw_ad9833_select(BSW_AD9833_REG_0);
            
            s_ctx.pre_linked_phase         = PRE_SELF_REPLY;
            s_ctx.pre_phase_enter_tick     = now_tick;
            s_ctx.self_reply_cur_is_f0     = true;
            s_ctx.self_reply_next_bit_tick = now_tick + PRE_BFSK_BIT_PERIOD_MS;
        }
        else
        {
            // 重发次数用尽，回到 SCAN_LISTEN
            s_ctx.uplink_freq.f0_hz  = FREQ_NOT_LOCKED;
            s_ctx.uplink_freq.f1_hz  = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f0_hz = FREQ_NOT_LOCKED;
            s_ctx.sweep_result.f1_hz = FREQ_NOT_LOCKED;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
        }
    }
}

// ============================================================================
// 状态运行函数实现
// ============================================================================

/**
 * @brief BOOT 状态运行函数
 * @param now_tick 当前时刻（ms）
 * @details BOOT 状态无运行逻辑，等待外部切换
 */
static void _state_run_boot(uint32_t now_tick)
{
    (void)now_tick;
}

/**
 * @brief SCAN_LISTEN 状态运行函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  监听上级扫频信号，850ms 会话硬超时检查：
 *  - 超时 → 完成扫频会话，触发 on_sweep_complete 回调
 */
static void _state_run_scan_listen(uint32_t now_tick)
{
    // 850 ms 会话硬超时检查（协议 §二 机制 A）
    if (s_ctx.sweep_start_tick != APP_NODE_FSM_SWEEP_NOT_STARTED
        && (now_tick - s_ctx.sweep_start_tick) >= SWEEP_SESSION_TIMEOUT_MS)
    {
        uint8_t cnt = 0;
        for (int i = 0; i < SWEEP_FREQ_COUNT; ++i)
        {
            if (s_ctx.sweep_amp_table[i] > APP_NODE_FSM_SWEEP_AMP_VALID_THRESHOLD)
            {
                ++cnt;
            }
        }
        s_ctx.sweep_start_tick   = APP_NODE_FSM_SWEEP_NOT_STARTED;
        s_ctx.sweep_timeout_tick = APP_NODE_FSM_SWEEP_NOT_STARTED;
        app_node_fsm_on_sweep_complete(cnt);
    }
}

/**
 * @brief SCAN 状态运行函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  主动扫频：发送 19 个频点
 *  - 每 SCAN_FREQ_DURATION_MS 切换一次频点
 *  - 发送完成后进入休眠，等待 850ms 硬超时
 *  - 超时后进入 SCAN_WAIT_REPLY 状态
 */
static void _state_run_scan(uint32_t now_tick)
{
    // 检查是否到达下一个频点切换时刻
    if ((int32_t)(now_tick - s_ctx.scan_freq_next_tick) >= 0)
    {
        // 移动到下一个频点
        s_ctx.scan_freq_index++;
        
        if (s_ctx.scan_freq_index < SWEEP_FREQ_COUNT)
        {
            // 计算并发送下一个频点
            uint16_t next_freq = SWEEP_FREQ_START + (s_ctx.scan_freq_index * SWEEP_FREQ_STEP);
            (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, next_freq);
            (void)bsw_ad9833_select(BSW_AD9833_REG_0);
            
            // 更新下一次切换时刻
            s_ctx.scan_freq_next_tick = now_tick + SCAN_FREQ_DURATION_MS;
        }
        else
        {
            // 19 个频点全部发送完毕，停止发送，进入 AD9833 休眠
            (void)bsw_ad9833_sleep(true);
            
            // 注：扫频发送完成后，不立即退出 SCAN 状态，
            // 而是等待 850ms 硬超时，让接收方有足够时间处理幅度表。
            // 超时后由下面的逻辑触发转移到 SCAN_WAIT_REPLY
        }
    }
    
    // 850 ms 会话硬超时检查
    if ((now_tick - s_ctx.sweep_start_tick) >= SWEEP_SESSION_TIMEOUT_MS)
    {
        // SCAN 状态（主动发送方）扫频结束
        // 进入 SCAN_WAIT_REPLY 状态，监听下级节点的 1010 回应波形
        _state_transition(NODE_SCAN_WAIT_REPLY, now_tick);
    }
}

/**
 * @brief SCAN_WAIT_REPLY 状态运行函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  上级节点等待下级的 1010 回应波形
 *  
 *  PHASE1 SCAN_WAIT_1010: 等待下级 1010 波形（ADC 频率检测）
 *    - 检测到有效 1010 → 发送 ACK → 进入 PHASE2
 *    - 超时 → 回到 SCAN_LISTEN
 *  
 *  PHASE2 SCAN_ACK_SENT: 已发送 ACK，确认等待（BFSK 解调器监听重复 1010）
 *    - 检测到重复 1010 → ACK 丢失，重发 ACK
 *    - 超时无 1010 → 确认通信建立，进入 LINKED
 *    - 重发次数用尽 → 回到 SCAN_LISTEN
 */
static void _state_run_scan_wait_reply(uint32_t now_tick)
{
    if (s_ctx.scan_wait_reply_phase == SCAN_WAIT_1010)
    {
        // PHASE1: 等待 1010 波形
        
        // 超时检查
        if ((now_tick - s_ctx.scan_wait_reply_start_tick) >= SCAN_WAIT_REPLY_TIMEOUT_MS)
        {
            // 超时未收到下级回应，回到 SCAN_LISTEN 状态
            bsw_adc_ringbuf_disable();
            _state_transition(NODE_SCAN_LISTEN, now_tick);
            return;
        }
        
        // 1010 检测由 app_node_fsm_on_sweep_detected() 处理
        // 检测到有效频对后会自动切换到 PHASE2
         
    }
    else if (s_ctx.scan_wait_reply_phase == SCAN_ACK_SENT)
    {
        // PHASE2: ACK 已发送，等待确认
        
        // 超时检查 - 如果超时无重复 1010，说明下级收到了 ACK
        if ((now_tick - s_ctx.scan_ack_sent_tick) >= SCAN_ACK_CONFIRM_TIMEOUT_MS)
        {
            // 确认通信建立成功，进入 LINKED
            mcal_timer_ic_stop(MCAL_TIMER_TIM1);
            _state_transition(NODE_LINKED, now_tick);
            return;
        }
        
        // 继续监听是否有重复的 1010 波形（使用 ADC 频率检测）
        // 注意：这里不使用 BFSK 解调器，因为 1010 只是载波切换，不是数据帧
        // 1010 检测逻辑由 on_sweep_detected 事件注入，通过 reply_toggle_count 判断
        
        // 如果检测到重复的 1010（reply_toggle_count >= REPLY_1010_MIN_TOGGLE_COUNT）
        // 说明下级没收到 ACK，需要重发
        if (s_ctx.reply_toggle_count >= REPLY_1010_MIN_TOGGLE_COUNT)
        {
            // 检测到重复的 1010
            
            // 检查重发次数
            if (s_ctx.scan_ack_retry_count < SCAN_ACK_MAX_RETRY)
            {
                // 等待 1010 发送完成（PRE_REPLY_DURATION_MS = 2000ms）
                // 确保总线空闲后再发送 ACK
                
                if (s_ctx.scan_1010_detected_tick == QUIET_NOT_STARTED)
                {
                    // 首次检测到 1010，记录时刻，开始等待
                    s_ctx.scan_1010_detected_tick = now_tick;
                }
                else if ((now_tick - s_ctx.scan_1010_detected_tick) >= PRE_REPLY_DURATION_MS)
                {
                    // 等待完成（2000ms 后），总线空闲，现在可以重发 ACK
                    s_ctx.scan_ack_retry_count++;
                    
                    // TODO: 发送 ACK 帧
                    // 载荷：[f0_hz(2)][f1_hz(2)] 小端序
                    
                    // 更新重发时刻
                    s_ctx.scan_ack_sent_tick = now_tick;
                    
                    // 重置检测标志，准备下一次检测
                    s_ctx.scan_1010_detected_tick = QUIET_NOT_STARTED;
                    s_ctx.reply_toggle_count = FLAG_CLEARED;
                    s_ctx.reply_last_freq = FREQ_NOT_LOCKED;
                }
                // else: 继续等待 1010 发送完成（还没到 2000ms）
            }
            else
            {
                // 重发次数用尽，清空下级频点，回到 SCAN_LISTEN
                s_ctx.downlink_freq.f0_hz = FREQ_NOT_LOCKED;
                s_ctx.downlink_freq.f1_hz = FREQ_NOT_LOCKED;
                s_ctx.sweep_result.f0_hz  = FREQ_NOT_LOCKED;
                s_ctx.sweep_result.f1_hz  = FREQ_NOT_LOCKED;
                _state_transition(NODE_SCAN_LISTEN, now_tick);
            }
        }
        // 其他情况继续等待
    }
}

/**
 * @brief PRE_LINKED 状态运行函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  调用子状态机处理函数（通过函数表）
 */
static void _state_run_pre_linked(uint32_t now_tick)
{
    phase_handler_t h = s_phase_handlers[s_ctx.pre_linked_phase];
    if (h != NULL)
    {
        h(now_tick);
    }
}

// ============================================================================
// LINKED 状态：帧处理（委托给 app_frame_handler）
// ============================================================================

/**
 * @brief 处理解调器接收到的 bit 流
 * 
 * @details
 *  调用链：demod → proto → frame_handler
 *  1. 取 demod 结果（bit 流）
 *  2. 喂给 proto 状态机（bit → 字节 → 帧）
 *  3. 解析成功后分发给 app_frame_handler
 *  4. 设置标志位
 */
static void _handle_demod_result(void)
{
    bsw_bfsk_demod_state_t demod_state = bsw_bfsk_demod_get_state();
    
    if (demod_state == BSW_BFSK_DEMOD_DONE_OK)
    {
        bsw_bfsk_demod_result_t demod_result;
        if (bsw_bfsk_demod_take_result(&demod_result) == BSW_BFSK_DEMOD_OK)
        {
            // 步骤 1: 初始化协议接收器（首次）
            static bool s_proto_rx_initialized = false;
            if (!s_proto_rx_initialized)
            {
                proto_rx_init(bsw_node_id_get());
                s_proto_rx_initialized = true;
            }
            
            // 步骤 2: 把 bit 流喂给协议层
            proto_err_t proto_err = proto_rx_feed_bits(demod_result.bit_buf, 
                                                       demod_result.bit_count);
            
            // 步骤 3: 检查协议层是否有解析好的帧
            if (proto_rx_get_state() == PROTO_RX_DONE)
            {
                proto_frame_t frame;
                if (proto_rx_take_frame(&frame) == PROTO_ERR_OK)
                {
                    // 步骤 4: 分发给业务层处理
                    app_frame_handler_dispatch(&frame);
                    
                    // 步骤 5: 设置标志位（链路活跃）
                    app_node_fsm_flag_set_rx_up();
                }
            }
            
            // 协议层错误处理
            if (proto_rx_get_state() == PROTO_RX_ERROR)
            {
                proto_err_t err = proto_rx_get_last_error();
                (void)err;  // TODO: 错误统计
            }
        }
    }
    else if (demod_state == BSW_BFSK_DEMOD_DONE_TIMEOUT)
    {
        bsw_bfsk_demod_stop();
        // 根据需要重新启动
    }
    else if (demod_state == BSW_BFSK_DEMOD_DONE_ERR)
    {
        // 解调错误（缓冲区满等），复位并重新启动
        bsw_bfsk_demod_stop();
    }
}

/**
 * @brief LINKED 状态运行函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  正常通信状态，处理解调数据和断链检测：
 *  1. 处理解调结果（bit 流 → 帧 → 业务）
 *  2. 断链检测（上行链路）
 *  3. 断链检测（下行链路）
 */
static void _state_run_linked(uint32_t now_tick)
{
    // 步骤 1: 处理解调结果（bit 流 → 帧 → 业务）
    _handle_demod_result();
    
    // ========== 断链检测（上行链路） ==========
    
    // 场景 2：所有标志位为 0（链路完全断裂，频点可能已变化）
    // - 收不到上级信号
    // - 也无下行通信活动
    // → 进入 SCAN_LISTEN 被动等待上级重新扫频
    // 注意：井上节点（无上级）不会触发此场景，因为它的 rx_up 和 fwd_up 永远为 1
    if ((s_ctx.flags.flag_rx_up_ok  == 0)
        && (s_ctx.flags.flag_fwd_dn_ok == 0)
        && (s_ctx.flags.flag_rx_dn_ok  == 0)
        && (s_ctx.flags.flag_fwd_up_ok == 0))
    {
        if (_is_bus_quiet(now_tick))
        {
            // 井上节点不会进入这里（rx_up 永远为 1）
            // 井下和中继节点：完全孤立，进入监听
            _state_transition(NODE_SCAN_LISTEN, now_tick);
            return;
        }
    }
    
    // 场景 3：有下行通信活动，但向上转发失败（上行链路中途断裂）
    // - 下级节点正常工作（能接收或转发给下级）
    // - 但无法转发给上级（上行频点可能变化）
    // → 进入 SCAN_LISTEN 重新监听上级新频点
    // 注意：井上节点（无上级）和井下节点（无下级）不会触发此场景
    if (app_node_fsm_has_uplink() && app_node_fsm_has_downlink())
    {
        // 仅中继节点检测此场景
        if (((s_ctx.flags.flag_fwd_dn_ok != 0) || (s_ctx.flags.flag_rx_dn_ok != 0))
            && (s_ctx.flags.flag_fwd_up_ok == 0))
        {
            if (_is_bus_quiet(now_tick))
            {
                _state_transition(NODE_SCAN_LISTEN, now_tick);
                return;
            }
        }
    }
    
    // ========== 断链检测（下行链路） ==========
    
    // 场景一：上行通畅但下行异常 → 进入 SCAN 主动扫频重建下行
    if (_check_link_failure_in_linked(now_tick) == LINK_FAIL_NEED_SCAN)
    {
        _state_transition(NODE_SCAN, now_tick);
    }
}

/**
 * @brief SLEEP 状态运行函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  休眠状态：检测是否有活动，有则回到监听状态
 */
static void _state_run_sleep(uint32_t now_tick)
{
    if (!app_node_fsm_is_clean())
    {
        _state_transition(NODE_SCAN_LISTEN, now_tick);
    }
}

/**
 * @brief FAULT 状态运行函数
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  故障状态：静默，等待外部复位
 */
static void _state_run_fault(uint32_t now_tick)
{
    (void)now_tick;
    // FAULT 状态静默，等待外部复位
}

// ============================================================================
// 公共 API 实现
// ============================================================================

/**
 * @brief 初始化节点状态机
 * @param hard_id 节点硬件地址
 * 
 * @details
 *  初始化流程：
 *  1. 清空上下文
 *  2. 设置节点 ID 和默认参数
 *  3. 初始化底层模块（LED、版本、存储）
 *  4. 从 Flash 读取频点备份
 *  5. 校验频点有效性（精确匹配频点表）
 *  6. 选择初始状态（LINKED 或 SCAN_LISTEN）
 */
void app_node_fsm_init(uint8_t hard_id)
{
    memset(&s_ctx, 0, sizeof(s_ctx));

    s_ctx.hard_id = hard_id;
    s_ctx.noise_floor = SWEEP_DEFAULT_NOISE_FLOOR;
    
    // 初始化频点为未锁定状态（Flash 读取失败时的默认值）
    s_ctx.uplink_freq.f0_hz   = FREQ_NOT_LOCKED;
    s_ctx.uplink_freq.f1_hz   = FREQ_NOT_LOCKED;
    s_ctx.downlink_freq.f0_hz = FREQ_NOT_LOCKED;
    s_ctx.downlink_freq.f1_hz = FREQ_NOT_LOCKED;

    // 初始化活动链路为未激活
    s_ctx.active_link_is_uplink = APP_NODE_FSM_LINK_NONE;

    bsw_node_id_init(hard_id);
    bsw_led_init();
    bsw_version_init();

    // 初始化 Flash 频点存储模块
    bsw_freq_storage_init();

    // 一次性读取上行 + 下行两对频点
    // - Flash 有备份 → 直接使用（快速恢复）
    // - Flash 无备份 → load_all 内部把所有输出置 0（即 FREQ_NOT_LOCKED）
    //
    // 注意：这里不直接进入 LINKED 状态，因为：
    // 1. 对端可能已经掉电重启，频点可能已变化
    // 2. 需要先通过扫频/监听重新建立通信
    // 3. 只是将频点作为"上次锁定频点"供诊断和优化使用
    //
    // 进入 LINKED 时会在 _state_entry_linked() 中校验频点有效性：
    // - 有效 → 正常进入 LINKED
    // - 无效 → 跳转到 SCAN/SCAN_LISTEN（双保险）
    uint16_t saved_uf0 = 0, saved_uf1 = 0, saved_df0 = 0, saved_df1 = 0;
    (void)bsw_freq_storage_load_all(&saved_uf0, &saved_uf1, &saved_df0, &saved_df1);
    // load_all 内部已做合理性校验（范围、非零、非 0xFFFF），失败时输出为 0
    
    // APP 层二次校验：精确匹配频点表（防止 Flash 垃圾数据）
    // BSW 层只做范围检查（100~2000 Hz），这里做精确表内校验
    // 只有频点对的两个频点都在表内才认为有效，否则清零
    bool uf0_in_table = (saved_uf0 != 0) && _is_freq_in_table(saved_uf0);
    bool uf1_in_table = (saved_uf1 != 0) && _is_freq_in_table(saved_uf1);
    bool df0_in_table = (saved_df0 != 0) && _is_freq_in_table(saved_df0);
    bool df1_in_table = (saved_df1 != 0) && _is_freq_in_table(saved_df1);
    
    if (uf0_in_table && uf1_in_table)
    {
        s_ctx.uplink_freq.f0_hz = saved_uf0;
        s_ctx.uplink_freq.f1_hz = saved_uf1;
    }
    else
    {
        s_ctx.uplink_freq.f0_hz = FREQ_NOT_LOCKED;
        s_ctx.uplink_freq.f1_hz = FREQ_NOT_LOCKED;
    }
    
    if (df0_in_table && df1_in_table)
    {
        s_ctx.downlink_freq.f0_hz = saved_df0;
        s_ctx.downlink_freq.f1_hz = saved_df1;
    }
    else
    {
        s_ctx.downlink_freq.f0_hz = FREQ_NOT_LOCKED;
        s_ctx.downlink_freq.f1_hz = FREQ_NOT_LOCKED;
    }

    // 根据 Flash 备份的频点情况选择初始状态
    _select_initial_state();
}

/**
 * @brief 状态机运行函数（周期调用）
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  使用函数表模式，根据当前状态调用对应的运行函数
 */
void app_node_fsm_run(uint32_t now_tick)
{
    // 状态运行函数表
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

    if (s_ctx.state < sizeof(s_run_table) / sizeof(s_run_table[0]))
    {
        state_run_t run_fn = s_run_table[s_ctx.state];
        if (run_fn != NULL)
        {
            run_fn(now_tick);
        }
    }
}

/**
 * @brief 获取当前状态
 * @return 当前状态机状态
 */
node_state_t app_node_fsm_get_state(void)
{
    return s_ctx.state;
}

/**
 * @brief 检查所有标志位是否清空
 * @return true = 所有标志位为 0（无活动），false = 有活动
 * 
 * @details
 *  用于判断总线是否完全静默，无任何通信活动
 */
bool app_node_fsm_is_clean(void)
{
    return (s_ctx.flags.flag_rx_up_ok  == false)
           && (s_ctx.flags.flag_fwd_dn_ok == false)
           && (s_ctx.flags.flag_rx_dn_ok  == false)
           && (s_ctx.flags.flag_fwd_up_ok == false);
}

/**
 * @brief 检查上行频点是否锁定
 * @return true = 已锁定，false = 未锁定
 */
bool app_node_fsm_is_uplink_locked(void)
{
    return (s_ctx.uplink_freq.f0_hz != FREQ_NOT_LOCKED);
}

/**
 * @brief 检查下行频点是否锁定
 * @return true = 已锁定，false = 未锁定
 */
bool app_node_fsm_is_downlink_locked(void)
{
    return (s_ctx.downlink_freq.f0_hz != FREQ_NOT_LOCKED);
}

/**
 * @brief 获取上行频点
 * @param[out] p_f0_hz 低频点（输出）
 * @param[out] p_f1_hz 高频点（输出）
 * 
 * @details
 *  如果指针为 NULL，则跳过该输出参数
 */
void app_node_fsm_get_uplink_freq(uint16_t *p_f0_hz, uint16_t *p_f1_hz)
{
    if (p_f0_hz) *p_f0_hz = s_ctx.uplink_freq.f0_hz;
    if (p_f1_hz) *p_f1_hz = s_ctx.uplink_freq.f1_hz;
}

/**
 * @brief 获取下行频点
 * @param[out] p_f0_hz 低频点（输出）
 * @param[out] p_f1_hz 高频点（输出）
 * 
 * @details
 *  如果指针为 NULL，则跳过该输出参数
 */
void app_node_fsm_get_downlink_freq(uint16_t *p_f0_hz, uint16_t *p_f1_hz)
{
    if (p_f0_hz) *p_f0_hz = s_ctx.downlink_freq.f0_hz;
    if (p_f1_hz) *p_f1_hz = s_ctx.downlink_freq.f1_hz;
}

/**
 * @brief 获取状态机上下文（只读）
 * @return 指向上下文的常量指针
 * 
 * @details
 *  用于调试和诊断，外部只能读取不能修改
 */
const app_node_ctx_t *app_node_fsm_get_ctx(void)
{
    return &s_ctx;
}

// ============================================================================
// 事件注入实现
// ============================================================================

/**
 * @brief 扫频检测事件注入
 * @param f_hz 检测到的频率（Hz）
 * @param amplitude 信号幅度
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  两个状态处理扫频检测：
 *  1. SCAN_LISTEN：下级节点监听上级的 19 频点扫频
 *  2. SCAN_WAIT_REPLY：上级节点监听下级的 1010 交替波形
 */
void app_node_fsm_on_sweep_detected(uint16_t f_hz, uint16_t amplitude, uint32_t now_tick)
{
    // SCAN_LISTEN 状态：下级节点监听上级的 19 频点扫频
    if (s_ctx.state == NODE_SCAN_LISTEN)
    {
        uint8_t idx = _sweep_freq_to_index(f_hz);
        if (idx == SWEEP_FREQ_INDEX_INVALID)
        {
            return;
        }

        _update_amp_table(idx, amplitude);

        // 首次命中 → 启动 850 ms 会话倒计时（协议 §二 机制 A）
        if (s_ctx.sweep_start_tick == APP_NODE_FSM_SWEEP_NOT_STARTED)
        {
            s_ctx.sweep_start_tick   = now_tick;
            s_ctx.sweep_timeout_tick = now_tick + SWEEP_SESSION_TIMEOUT_MS;
        }

        s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
        return;
    }
    
    // SCAN_WAIT_REPLY 状态：上级节点监听下级的 1010 交替波形
    if (s_ctx.state == NODE_SCAN_WAIT_REPLY)
    {
        // 检测 1010 交替模式：f0 ↔ f1 ↔ f0 ↔ f1
        // 策略：记录最近两个不同的频率，检测它们的交替切换
        
        // 忽略振幅过低的信号（可能是噪声）
        if (amplitude < APP_NODE_FSM_SWEEP_AMP_VALID_THRESHOLD + REPLY_MIN_AMPLITUDE_MARGIN)
        {
            return;
        }
        
        // 第一次检测到频率
        if (s_ctx.reply_detected_f0 == FREQ_NOT_LOCKED)
        {
            s_ctx.reply_detected_f0 = f_hz;
            s_ctx.reply_last_freq   = f_hz;
            s_ctx.reply_toggle_count = 0;
            return;
        }
        
        // 检测到与 f0 不同的频率 → 作为 f1
        if (s_ctx.reply_detected_f1 == FREQ_NOT_LOCKED)
        {
            // 频率差异要足够大（至少 100Hz），避免噪声干扰
            int32_t freq_diff = (int32_t)f_hz - (int32_t)s_ctx.reply_detected_f0;
            if (freq_diff < -(int32_t)REPLY_FREQ_DIFF_MIN_HZ || freq_diff > (int32_t)REPLY_FREQ_DIFF_MIN_HZ)
            {
                s_ctx.reply_detected_f1 = f_hz;
                s_ctx.reply_last_freq   = f_hz;
                s_ctx.reply_toggle_count = FLAG_SET;  // 第一次切换
            }
            return;
        }
        
        // 已经检测到 f0 和 f1，现在检测交替模式
        // 判断当前频率应该是 f0 还是 f1（容差 ±30Hz）
        int32_t diff_to_f0 = (int32_t)f_hz - (int32_t)s_ctx.reply_detected_f0;
        int32_t diff_to_f1 = (int32_t)f_hz - (int32_t)s_ctx.reply_detected_f1;
        
        if (diff_to_f0 < 0) diff_to_f0 = -diff_to_f0;
        if (diff_to_f1 < 0) diff_to_f1 = -diff_to_f1;
        
        uint16_t current_freq;
        if (diff_to_f0 < diff_to_f1 && diff_to_f0 < REPLY_FREQ_TOLERANCE_HZ)
        {
            current_freq = s_ctx.reply_detected_f0;
        }
        else if (diff_to_f1 < REPLY_FREQ_TOLERANCE_HZ)
        {
            current_freq = s_ctx.reply_detected_f1;
        }
        else
        {
            // 频率偏差太大，可能是噪声，重置检测
            s_ctx.reply_detected_f0  = FREQ_NOT_LOCKED;
            s_ctx.reply_detected_f1  = FREQ_NOT_LOCKED;
            s_ctx.reply_toggle_count = 0;
            return;
        }
        
        // 检测到切换（当前频率 != 上次频率）
        if (current_freq != s_ctx.reply_last_freq)
        {
            s_ctx.reply_toggle_count++;
            s_ctx.reply_last_freq = current_freq;
            
            // 连续检测到 6 次交替切换（f0→f1→f0→f1→f0→f1→f0），确认是 1010 模式
            if (s_ctx.reply_toggle_count >= REPLY_1010_MIN_TOGGLE_COUNT)
            {
                // 只在 PHASE1 处理 1010 检测
                if (s_ctx.scan_wait_reply_phase != SCAN_WAIT_1010)
                {
                    return;
                }
                
                // 确定 f0 < f1
                uint16_t f0 = s_ctx.reply_detected_f0;
                uint16_t f1 = s_ctx.reply_detected_f1;
                if (f0 > f1)
                {
                    uint16_t tmp = f0;
                    f0 = f1;
                    f1 = tmp;
                }
                
                // 验证频率间隔 >= 150Hz
                if ((f1 - f0) < SWEEP_MIN_FREQ_GAP_HZ)
                {
                    // 频率间隔太小，重置检测
                    s_ctx.reply_detected_f0  = FREQ_NOT_LOCKED;
                    s_ctx.reply_detected_f1  = FREQ_NOT_LOCKED;
                    s_ctx.reply_toggle_count = 0;
                    return;
                }
                
                // 成功检测到有效频对！保存到上下文
                // 扫频方：建立的是下级频点（与下级通信）
                s_ctx.downlink_freq.f0_hz = f0;
                s_ctx.downlink_freq.f1_hz = f1;
                s_ctx.sweep_result.f0_hz = f0;
                s_ctx.sweep_result.f1_hz = f1;
                s_ctx.active_link_is_uplink = LINK_DOWNLINK;
                
                // 备份频对到 Flash（扫频方刚确认下行频点，保留上行原值）
                int32_t ret = bsw_freq_storage_save_all(
                    s_ctx.uplink_freq.f0_hz, s_ctx.uplink_freq.f1_hz,
                    f0, f1);
                (void)ret;  // 备份失败不影响通信建立，只是下次掉电无法快速恢复
                
                // TODO: 发送 ACK 帧（PROTO_TYPE_REPLY + PROTO_REPLY_FREQ_PAIR）
                // 载荷：[f0_hz(2)][f1_hz(2)] 小端序
                // 暂时先切换到 PHASE2，等协议层完善后再实现 ACK 发送
                
                // 停止 ADC，启动 BFSK 解调器监听重复的 1010
                bsw_adc_ringbuf_disable();
                mcal_timer_ic_start(MCAL_TIMER_TIM1);
                bsw_bfsk_demod_start(f0, f1, PRE_BFSK_BIT_PERIOD_MS, SCAN_ACK_CONFIRM_TIMEOUT_MS, s_ctx.hard_id);
                
                // 切换到 PHASE2: ACK 已发送，等待确认
                s_ctx.scan_wait_reply_phase = SCAN_ACK_SENT;
                s_ctx.scan_ack_sent_tick    = now_tick;
                s_ctx.scan_ack_retry_count  = 0;
            }
        }
        
        return;
    }
}

/**
 * @brief 扫频完成事件注入
 * @param valid_freq_count 有效频点数量
 * 
 * @details
 *  扫频完成后的处理流程：
 *  1. 调用频对决选算法选出最佳频对
 *  2. 选择失败 → 回到 SCAN_LISTEN
 *  3. 选择成功 → 保存频对，进入 PRE_LINKED 状态
 */
void app_node_fsm_on_sweep_complete(uint8_t valid_freq_count)
{
    uint16_t best_f0 = 0, best_f1 = 0;
    int32_t dr = _select_freq_pair(&best_f0, &best_f1);
    
    if (dr == -1)
    {
        _state_transition(NODE_SCAN_LISTEN, HAL_GetTick());
        return;
    }
    
    if (dr == -2)
    {
        _state_transition(NODE_SCAN_LISTEN, HAL_GetTick());
        return;
    }

    // 成功找到频点对
    s_ctx.sweep_result.f0_hz = best_f0;
    s_ctx.sweep_result.f1_hz = best_f1;
    
    // 被扫方：建立的是上级频点（与上级通信）
    s_ctx.uplink_freq.f0_hz = best_f0;
    s_ctx.uplink_freq.f1_hz = best_f1;
    s_ctx.active_link_is_uplink = LINK_UPLINK;

    // 扫频完成 → 进入预链接状态（协议 §4.2）
    // 已选出最佳频对 (f0, f1)，后续工作（协议 §4.2.1 自载波 1010 回显、
    // §4.2.3 等 ACK、§4.2.3 超时回退）在 PRE_LINKED 状态中处理
    _state_transition(NODE_PRE_LINKED, HAL_GetTick());
}

/**
 * @brief ACK 接收事件注入
 * @param f0 ACK 中的低频点
 * @param f1 ACK 中的高频点
 * 
 * @details
 *  被扫方收到上级 ACK，确认上级频点并进入 LINKED 状态
 */
void app_node_fsm_on_ack_received(uint16_t f0, uint16_t f1)
{
    if (s_ctx.state != NODE_SCAN)
    {
        return;
    }

    // 被扫方收到上级 ACK，确认上级频点
    s_ctx.uplink_freq.f0_hz = f0;
    s_ctx.uplink_freq.f1_hz = f1;
    s_ctx.active_link_is_uplink = LINK_UPLINK;

    // 备份频对到 Flash（被扫方刚确认上行频点，保留下行原值）
    int32_t ret = bsw_freq_storage_save_all(
        f0, f1,
        s_ctx.downlink_freq.f0_hz, s_ctx.downlink_freq.f1_hz);
    (void)ret;  // 备份失败不影响进入 LINKED，只是下次掉电无法快速恢复

    _state_transition(NODE_LINKED, HAL_GetTick());
}

// ============================================================================
// 事务标志位操作
// ============================================================================

/**
 * @brief 设置上行接收标志
 * @details 收到上级信号时调用，重置静默计时器
 */
void app_node_fsm_flag_set_rx_up(void)
{
    s_ctx.flags.flag_rx_up_ok = FLAG_SET;
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

/**
 * @brief 设置下行转发标志
 * @details 成功转发给下级时调用，重置静默计时器
 */
void app_node_fsm_flag_set_fwd_dn(void)
{
    s_ctx.flags.flag_fwd_dn_ok = FLAG_SET;
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

/**
 * @brief 设置下行接收标志
 * @details 收到下级信号时调用，重置静默计时器
 */
void app_node_fsm_flag_set_rx_dn(void)
{
    s_ctx.flags.flag_rx_dn_ok = FLAG_SET;
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

/**
 * @brief 设置上行转发标志
 * @details 成功转发给上级时调用，重置静默计时器
 */
void app_node_fsm_flag_set_fwd_up(void)
{
    s_ctx.flags.flag_fwd_up_ok = FLAG_SET;
    s_ctx.quiet_start_tick = QUIET_NOT_STARTED;
}

/**
 * @brief 清除上行接收标志
 */
void app_node_fsm_flag_clear_rx_up(void)
{
    s_ctx.flags.flag_rx_up_ok = FLAG_CLEARED;
}

/**
 * @brief 清除下行转发标志
 */
void app_node_fsm_flag_clear_fwd_dn(void)
{
    s_ctx.flags.flag_fwd_dn_ok = FLAG_CLEARED;
}

/**
 * @brief 清除下行接收标志
 */
void app_node_fsm_flag_clear_rx_dn(void)
{
    s_ctx.flags.flag_rx_dn_ok = FLAG_CLEARED;
}

/**
 * @brief 清除上行转发标志
 */
void app_node_fsm_flag_clear_fwd_up(void)
{
    s_ctx.flags.flag_fwd_up_ok = FLAG_CLEARED;
}

/**
 * @brief 扫频命令事件注入
 * @param scan_direction 扫频方向（0=扫上行，1=扫下行）
 * 
 * @details
 *  【中心化扫频控制】
 *  - 井上通过 CMD_SCAN_ORDER 指令指定节点扫频
 *  - scan_direction: 0=扫上行（一般不用），1=扫下行
 *  
 *  扫频方向：
 *  - 1 = 扫下行：设置活动链路为下行，进入 SCAN 状态
 *  - 0 = 扫上行：设置活动链路为上行，进入 SCAN_LISTEN 状态
 */
void app_node_fsm_on_scan_order(uint8_t scan_direction)
{
    uint32_t now_tick = HAL_GetTick();
    
    if (scan_direction == 1)
    {
        // 扫下行：设置活动链路为下行，进入 SCAN 状态
        if (app_node_fsm_has_downlink())
        {
            s_ctx.active_link_is_uplink = LINK_DOWNLINK;
            _state_transition(NODE_SCAN, now_tick);
        }
        // 井下节点没有下级，忽略此命令
    }
    else
    {
        // 扫上行：一般不需要，因为上行断了应该自动进入 SCAN_LISTEN
        // 但为了协议完整性，也支持这个方向
        if (app_node_fsm_has_uplink())
        {
            s_ctx.active_link_is_uplink = LINK_UPLINK;
            _state_transition(NODE_SCAN_LISTEN, now_tick);
        }
        // 井上节点没有上级，忽略此命令
    }
}

/**
 * @brief 静默超时事件注入
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  总线长时间无活动后触发：
 *  - 所有标志位清空 → 进入 SLEEP 状态
 *  - 有标志位置位 → 进入 SCAN_LISTEN 状态
 */
void app_node_fsm_on_quiet_timeout(uint32_t now_tick)
{
    if (app_node_fsm_is_clean())
    {
        s_ctx.quiet_start_tick = now_tick;
        _state_transition(NODE_SLEEP, now_tick);
    }
    else
    {
        _state_transition(NODE_SCAN_LISTEN, now_tick);
    }
}

/**
 * @brief 故障事件注入
 * @param fault_code 故障码
 * 
 * @details
 *  系统检测到故障时调用，进入 FAULT 状态
 */
void app_node_fsm_on_fault(uint32_t fault_code)
{
    (void)fault_code;
    _state_transition(NODE_FAULT, HAL_GetTick());
}

/**
 * @brief 看门狗超时事件注入
 * 
 * @details
 *  看门狗超时，系统异常，进入 FAULT 状态
 */
void app_node_fsm_on_wdt_timeout(void)
{
    _state_transition(NODE_FAULT, HAL_GetTick());
}

// ============================================================================
// 节点角色判断
// ============================================================================

/**
 * @brief 获取节点角色
 * @return 节点角色（井上/井下/中继）
 * 
 * @details
 *  根据硬件地址判断节点角色：
 *  - BSW_NODE_ADDR_GROUND (0) → 井上节点
 *  - BSW_NODE_ADDR_UNDERGROUND (0xF) → 井下节点
 *  - 其他 → 中继节点
 */
node_role_t app_node_fsm_get_role(void)
{
    if (s_ctx.hard_id == BSW_NODE_ADDR_GROUND)
    {
        return NODE_ROLE_SURFACE;  // 井上节点
    }
    else if (s_ctx.hard_id == BSW_NODE_ADDR_UNDERGROUND)
    {
        return NODE_ROLE_UNDERGROUND;  // 井下节点
    }
    else
    {
        return NODE_ROLE_RELAY;  // 中继节点
    }
}

/**
 * @brief 判断节点是否有上级
 * @return true = 有上级，false = 无上级
 * 
 * @details
 *  井上节点（BSW_NODE_ADDR_GROUND = 0）无上级，其余都有上级
 */
bool app_node_fsm_has_uplink(void)
{
    return (s_ctx.hard_id != BSW_NODE_ADDR_GROUND);
}

/**
 * @brief 判断节点是否有下级
 * @return true = 有下级，false = 无下级
 * 
 * @details
 *  井下节点（BSW_NODE_ADDR_UNDERGROUND = 0xF）无下级，其余都有下级
 */
bool app_node_fsm_has_downlink(void)
{
    return (s_ctx.hard_id != BSW_NODE_ADDR_UNDERGROUND);
}
