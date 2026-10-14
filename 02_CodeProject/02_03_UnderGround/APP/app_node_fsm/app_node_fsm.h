/**
 * @file    app_node_fsm.h
 * @brief   井下节点状态机模块 - 公共接口
 * 
 * @details
 *  本模块实现井下通信节点的状态机，支持：
 *  - 自动扫频和频点锁定
 *  - 上行/下行链路管理
 *  - 断链检测和自动恢复
 *  - 三种节点角色：井上、井下、中继
 *  
 *  使用流程：
 *  1. 调用 app_node_fsm_init() 初始化
 *  2. 周期调用 app_node_fsm_run() 运行状态机
 *  3. 通过事件注入 API 注入外部事件
 *  4. 通过查询 API 获取状态和频点信息
 */

#ifndef APP_NODE_FSM_H
#define APP_NODE_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 头文件依赖
// ============================================================================

#include <stdint.h>
#include <stdbool.h>
#include "mcal_timer.h"

// ============================================================================
// 节点状态枚举
// ============================================================================

/**
 * @brief 节点状态机状态
 * @details 状态转换流程：
 *  BOOT → SCAN_LISTEN ↔ SCAN → PRE_LINKED → LINKED ↔ SLEEP/FAULT
 */
typedef enum {
    NODE_BOOT = 0,              // 启动状态
    NODE_SCAN_LISTEN,           // 被动监听上级扫频
    NODE_SCAN,                  // 主动扫频下级
    NODE_SCAN_WAIT_REPLY,       // 等待下级回应
    NODE_PRE_LINKED,            // 预链接（回发频对）
    NODE_LINKED,                // 已链接（正常通信）
    NODE_SLEEP,                 // 休眠状态
    NODE_FAULT                  // 故障状态
} node_state_t;

// ============================================================================
// 事务标志位结构体
// ============================================================================

/**
 * @brief 4 标志位事务上下文
 * @details 用于跟踪链路活动状态：
 *  - flag_rx_up_ok:  收到上级信号
 *  - flag_fwd_dn_ok: 成功转发给下级
 *  - flag_rx_dn_ok:  收到下级信号
 *  - flag_fwd_up_ok: 成功转发给上级
 */
typedef struct {
    uint8_t flag_rx_up_ok  : 1;  // 上行接收标志
    uint8_t flag_fwd_dn_ok : 1;  // 下行转发标志
    uint8_t flag_rx_dn_ok  : 1;  // 下行接收标志
    uint8_t flag_fwd_up_ok : 1;  // 上行转发标志
} node_xact_flags_t;

// ============================================================================
// 频点对结构体
// ============================================================================

/**
 * @brief 工作频点对
 * @details BFSK 双频通信的频点对 (f0, f1)
 */
typedef struct {
    uint16_t f0_hz;  // 低频点（Hz）
    uint16_t f1_hz;  // 高频点（Hz）
} node_freq_pair_t;

// ============================================================================
// 扫频参数宏定义
// ============================================================================

#define SWEEP_FREQ_COUNT         (19)    // 扫频频点数量
#define SWEEP_FREQ_START         (125)   // 起始频率（Hz）
#define SWEEP_FREQ_STEP          (50)    // 频率步进（Hz）
#define SWEEP_FREQ_TOLERANCE_HZ  (15)    // 频率容差（Hz）
#define SWEEP_FREQ_INDEX_INVALID (0xFFU) // 无效索引

// ============================================================================
// 频对决选参数宏定义
// ============================================================================

#define SWEEP_MIN_FREQ_GAP_HZ                   (100u)  // 最小频率间隔（Hz）
#define SWEEP_VALID_AMP_DELTA                   (50u)   // 有效幅值差（保留）
#define SWEEP_DEFAULT_NOISE_FLOOR               (100u)  // 默认噪声底（保留）
#define SWEEP_MIN_VALID_FREQ_CNT                (10u)   // 最少有效频点数
#define SWEEP_AMP_STRENGTH_THRESHOLD_PERCENT    (70u)   // 强度门限（最大幅值的 70%）

#define SWEEP_SESSION_TIMEOUT_MS                (850U)  // 扫频会话超时（ms）

// ============================================================================
// 时间常量宏定义
// ============================================================================

#define BUS_QUIET_TIMEOUT_MS        (3000U)  // 总线静默超时（ms）
#define ADC_CHECK_PERIOD_MS         (40U)    // ADC 检查周期（ms）
#define SCAN_FREQ_DURATION_MS       (40U)    // 单个频点发送时长（ms）
#define SCAN_WAIT_REPLY_TIMEOUT_MS  (5000U)  // 等待回应超时（ms）
#define SCAN_ACK_CONFIRM_TIMEOUT_MS (6000U)  // ACK 确认超时（ms）
#define SCAN_ACK_MAX_RETRY          (2U)     // ACK 最大重发次数

// ============================================================================
// 节点角色枚举
// ============================================================================

/**
 * @brief 节点角色定义
 * @details 根据硬件地址判断节点在网络中的角色
 */
typedef enum {
    NODE_ROLE_SURFACE    = 0,  // 井上节点（地面站）
    NODE_ROLE_RELAY      = 1,  // 中继节点
    NODE_ROLE_UNDERGROUND = 2  // 井下节点（终端）
} node_role_t;

// ============================================================================
// 链路方向常量
// ============================================================================

#define LINK_UPLINK   (1U)  // 上行链路（向井上）
#define LINK_DOWNLINK (0U)  // 下行链路（向井下）

// ============================================================================
// 标志位状态常量
// ============================================================================

#define FLAG_SET     (1U)  // 标志位置位
#define FLAG_CLEARED (0U)  // 标志位清零

// ============================================================================
// 频率状态常量
// ============================================================================

#define FREQ_NOT_LOCKED (0U)  // 频率未锁定

// ============================================================================
// 时刻状态常量
// ============================================================================

#define QUIET_NOT_STARTED (0U)  // 静默计时未开始

// ============================================================================
// 1010 检测参数宏定义
// ============================================================================

#define REPLY_1010_MIN_BIT_COUNT    (8U)   // 1010 最少位数
#define REPLY_1010_MIN_TOGGLE_COUNT (6U)   // 1010 最少切换次数
#define REPLY_MIN_AMPLITUDE_MARGIN  (50U)  // 最小幅度裕度
#define REPLY_FREQ_DIFF_MIN_HZ      (100U) // 最小频率差（Hz）
#define REPLY_FREQ_TOLERANCE_HZ     (30U)  // 频率容差（Hz）

// ============================================================================
// PRE_LINKED 参数宏定义
// ============================================================================

#define PRE_QUIET_OBSERVE_MAX_MS (4000U)  // 静默观察最大时长（ms）
#define PRE_REPLY_DURATION_MS    (1000U)  // 回应发送时长（ms）
#define PRE_ACK_TIMEOUT_MS       (5000U)  // ACK 超时（ms）
#define PRE_ACK_MAX_RETRY        (1U)     // ACK 最大重试次数
#define PRE_BFSK_BIT_PERIOD_MS   (40U)    // BFSK 码元周期（ms）

/**
 * @brief 预链接子状态
 * @details PRE_LINKED 状态的内部阶段
 */
typedef enum {
    PRE_QUIET_OBSERVE = 0,  // 静默观察，等待时间槽
    PRE_SELF_REPLY    = 1,  // 回发频对（1010 波形）
    PRE_WAIT_ACK      = 2,  // 等待 ACK 确认
} pre_linked_phase_t;

/**
 * @brief SCAN_WAIT_REPLY 子阶段
 * @details SCAN_WAIT_REPLY 状态的内部阶段
 */
typedef enum {
    SCAN_WAIT_1010 = 0,  // 等待 1010 波形
    SCAN_ACK_SENT  = 1,  // ACK 已发送，等待确认
} scan_wait_reply_phase_t;

// ============================================================================
// 节点全局上下文结构体
// ============================================================================

/**
 * @brief 节点状态机全局上下文
 * @details 包含状态机运行所需的所有状态信息
 */
typedef struct {
    // 状态机基本信息
    node_state_t       state;              // 当前状态
    uint32_t           state_enter_tick;   // 进入当前状态的时刻（ms）

    // 事务标志位
    node_xact_flags_t  flags;              // 链路活动标志
    uint8_t            pending_buffer[64]; // 待发送缓冲（保留）
    uint16_t           pending_len;        // 待发送长度（保留）

    // 扫频相关
    node_freq_pair_t   sweep_result;                     // 扫频结果频对
    uint16_t           sweep_amp_table[SWEEP_FREQ_COUNT]; // 幅度表
    uint32_t           sweep_start_tick;                  // 扫频开始时刻（ms）
    uint32_t           sweep_timeout_tick;                // 扫频超时时刻（ms）

    // SCAN 状态相关
    uint8_t            scan_freq_index;       // 当前扫频索引
    uint32_t           scan_freq_next_tick;   // 下一个频点切换时刻（ms）
    
    // SCAN_WAIT_REPLY 状态相关
    uint32_t           scan_wait_reply_start_tick;     // 等待回应开始时刻（ms）
    scan_wait_reply_phase_t scan_wait_reply_phase;     // 等待回应子阶段
    uint32_t           scan_ack_sent_tick;             // ACK 发送时刻（ms）
    uint8_t            scan_ack_retry_count;           // ACK 重发次数
    uint32_t           scan_1010_detected_tick;        // 检测到 1010 的时刻（ms）
    uint16_t           reply_last_freq;                // 上次检测到的频率（Hz）
    uint16_t           reply_detected_f0;              // 检测到的低频点（Hz）
    uint16_t           reply_detected_f1;              // 检测到的高频点（Hz）
    uint8_t            reply_toggle_count;             // 频率切换计数

    // 链路频点
    node_freq_pair_t   uplink_freq;          // 上行频点
    node_freq_pair_t   downlink_freq;        // 下行频点
    uint8_t            active_link_is_uplink; // 活动链路方向（1=上行，0=下行）
    uint16_t           noise_floor;           // 噪声底（保留）
    uint8_t            hard_id;               // 节点硬件地址

    // 静默检测
    uint32_t           quiet_start_tick;      // 静默开始时刻（ms）

    // PRE_LINKED 状态相关
    uint32_t           pre_linked_last_check_tick;  // 上次检查时刻（ms）
    pre_linked_phase_t pre_linked_phase;            // 预链接子阶段
    uint32_t           pre_phase_enter_tick;        // 进入子阶段时刻（ms）
    uint8_t            self_reply_cur_is_f0;        // 当前发送的是 f0（1）还是 f1（0）
    uint32_t           self_reply_next_bit_tick;    // 下一个码元切换时刻（ms）
    uint8_t            pre_ack_retry_count;         // 预链接 ACK 重试次数
} app_node_ctx_t;

// ============================================================================
// 公共 API - 初始化和运行
// ============================================================================

/**
 * @brief 初始化节点状态机
 * @param hard_id 节点硬件地址
 * 
 * @details
 *  初始化流程：
 *  1. 清空上下文
 *  2. 从 Flash 读取频点备份
 *  3. 校验频点有效性
 *  4. 选择初始状态（LINKED 或 SCAN_LISTEN）
 */
void app_node_fsm_init(uint8_t hard_id);

/**
 * @brief 状态机运行函数（周期调用）
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  应在主循环中周期调用，建议周期：10~50ms
 */
void app_node_fsm_run(uint32_t now_tick);

// ============================================================================
// 公共 API - 状态查询
// ============================================================================

/**
 * @brief 获取当前状态
 * @return 当前状态机状态
 */
node_state_t app_node_fsm_get_state(void);

/**
 * @brief 检查所有标志位是否清空
 * @return true = 所有标志位为 0（无活动），false = 有活动
 */
bool app_node_fsm_is_clean(void);

/**
 * @brief 检查上行频点是否锁定
 * @return true = 已锁定，false = 未锁定
 */
bool app_node_fsm_is_uplink_locked(void);

/**
 * @brief 检查下行频点是否锁定
 * @return true = 已锁定，false = 未锁定
 */
bool app_node_fsm_is_downlink_locked(void);

/**
 * @brief 获取上行频点
 * @param[out] p_f0_hz 低频点（输出，可为 NULL）
 * @param[out] p_f1_hz 高频点（输出，可为 NULL）
 */
void app_node_fsm_get_uplink_freq(uint16_t *p_f0_hz, uint16_t *p_f1_hz);

/**
 * @brief 获取下行频点
 * @param[out] p_f0_hz 低频点（输出，可为 NULL）
 * @param[out] p_f1_hz 高频点（输出，可为 NULL）
 */
void app_node_fsm_get_downlink_freq(uint16_t *p_f0_hz, uint16_t *p_f1_hz);

/**
 * @brief 获取状态机上下文（只读）
 * @return 指向上下文的常量指针
 * 
 * @details
 *  用于调试和诊断，外部只能读取不能修改
 */
const app_node_ctx_t *app_node_fsm_get_ctx(void);

// ============================================================================
// 公共 API - 节点角色查询
// ============================================================================

/**
 * @brief 获取节点角色
 * @return 节点角色（井上/井下/中继）
 */
node_role_t app_node_fsm_get_role(void);

/**
 * @brief 判断节点是否有上级
 * @return true = 有上级，false = 无上级（井上节点）
 */
bool app_node_fsm_has_uplink(void);

/**
 * @brief 判断节点是否有下级
 * @return true = 有下级，false = 无下级（井下节点）
 */
bool app_node_fsm_has_downlink(void);

// ============================================================================
// 事件注入 API - 扫频相关
// ============================================================================

/**
 * @brief 扫频检测事件注入
 * @param f_hz 检测到的频率（Hz）
 * @param amplitude 信号幅度
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  当 ADC 检测到有效频率时调用，用于：
 *  - SCAN_LISTEN 状态：累积幅度表
 *  - SCAN_WAIT_REPLY 状态：检测 1010 波形
 */
void app_node_fsm_on_sweep_detected(uint16_t f_hz, uint16_t amplitude, uint32_t now_tick);

/**
 * @brief 扫频完成事件注入
 * @param valid_freq_count 有效频点数量
 * 
 * @details
 *  扫频会话超时后调用，触发频对决选算法
 */
void app_node_fsm_on_sweep_complete(uint8_t valid_freq_count);

/**
 * @brief ACK 接收事件注入
 * @param f0 ACK 中的低频点（Hz）
 * @param f1 ACK 中的高频点（Hz）
 * 
 * @details
 *  收到上级 ACK 确认后调用，确认频点并进入 LINKED 状态
 */
void app_node_fsm_on_ack_received(uint16_t f0, uint16_t f1);

// ============================================================================
// 事件注入 API - 标志位操作
// ============================================================================

/**
 * @brief 设置上行接收标志
 * @details 收到上级信号时调用
 */
void app_node_fsm_flag_set_rx_up(void);

/**
 * @brief 设置下行转发标志
 * @details 成功转发给下级时调用
 */
void app_node_fsm_flag_set_fwd_dn(void);

/**
 * @brief 设置下行接收标志
 * @details 收到下级信号时调用
 */
void app_node_fsm_flag_set_rx_dn(void);

/**
 * @brief 设置上行转发标志
 * @details 成功转发给上级时调用
 */
void app_node_fsm_flag_set_fwd_up(void);

/**
 * @brief 清除上行接收标志
 */
void app_node_fsm_flag_clear_rx_up(void);

/**
 * @brief 清除下行转发标志
 */
void app_node_fsm_flag_clear_fwd_dn(void);

/**
 * @brief 清除下行接收标志
 */
void app_node_fsm_flag_clear_rx_dn(void);

/**
 * @brief 清除上行转发标志
 */
void app_node_fsm_flag_clear_fwd_up(void);

// ============================================================================
// 事件注入 API - 系统事件
// ============================================================================

/**
 * @brief 静默超时事件注入
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  总线长时间无活动时调用，触发休眠或重新监听
 */
void app_node_fsm_on_quiet_timeout(uint32_t now_tick);

/**
 * @brief 故障事件注入
 * @param fault_code 故障码
 * 
 * @details
 *  系统检测到故障时调用，进入 FAULT 状态
 */
void app_node_fsm_on_fault(uint32_t fault_code);

/**
 * @brief 看门狗超时事件注入
 * 
 * @details
 *  看门狗超时时调用，进入 FAULT 状态
 */
void app_node_fsm_on_wdt_timeout(void);

/**
 * @brief 扫频命令事件注入
 * @param scan_direction 扫频方向（0=扫上行，1=扫下行）
 * 
 * @details
 *  上位机发送扫频命令时调用，中心化扫频控制
 */
void app_node_fsm_on_scan_order(uint8_t scan_direction);

#ifdef __cplusplus
}
#endif

#endif /* APP_NODE_FSM_H */
