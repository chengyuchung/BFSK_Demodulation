/**
 * @file    app_node_fsm.h
 * @brief   井下节点状态机模块
 */

#ifndef APP_NODE_FSM_H
#define APP_NODE_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "mcal_timer.h"

/* 节点状态枚举 */

typedef enum {
    NODE_BOOT = 0,
    NODE_SCAN_LISTEN,
    NODE_SCAN,
    NODE_SCAN_WAIT_REPLY,
    NODE_PRE_LINKED,
    NODE_LINKED,
    NODE_SLEEP,
    NODE_FAULT
} node_state_t;

/* 4 标志位事务上下文 */
typedef struct {
    uint8_t flag_rx_up_ok  : 1;
    uint8_t flag_fwd_dn_ok : 1;
    uint8_t flag_rx_dn_ok  : 1;
    uint8_t flag_fwd_up_ok : 1;
} node_xact_flags_t;

/* 工作频对 */
typedef struct {
    uint16_t f0_hz;
    uint16_t f1_hz;
} node_freq_pair_t;

/* 扫频参数 */
#define SWEEP_FREQ_COUNT         19
#define SWEEP_FREQ_START         125
#define SWEEP_FREQ_STEP          50
#define SWEEP_FREQ_TOLERANCE_HZ  15
#define SWEEP_FREQ_INDEX_INVALID 0xFFU

/* 频对决选参数 */
#define SWEEP_MIN_FREQ_GAP_HZ    150u
#define SWEEP_VALID_AMP_DELTA    50u
#define SWEEP_DEFAULT_NOISE_FLOOR 100u
#define SWEEP_MIN_VALID_FREQ_CNT 2u
#define SWEEP_SESSION_TIMEOUT_MS 850U

/* 时间常量 */
#define BUS_QUIET_TIMEOUT_MS        3000U
#define ADC_CHECK_PERIOD_MS         40U
#define SCAN_FREQ_DURATION_MS       40U
#define SCAN_WAIT_REPLY_TIMEOUT_MS  5000U
#define SCAN_ACK_CONFIRM_TIMEOUT_MS 6000U
#define SCAN_ACK_MAX_RETRY          2U

/* 节点角色定义 */
typedef enum {
    NODE_ROLE_SURFACE    = 0,
    NODE_ROLE_RELAY      = 1,
    NODE_ROLE_UNDERGROUND = 2
} node_role_t;

/* 链路方向常量 */
#define LINK_UPLINK   1U
#define LINK_DOWNLINK 0U

/* 标志位状态常量 */
#define FLAG_SET     1U
#define FLAG_CLEARED 0U

/* 频率状态常量 */
#define FREQ_NOT_LOCKED 0U

/* 时刻状态常量 */
#define QUIET_NOT_STARTED 0U

/* 1010 检测参数 */
#define REPLY_1010_MIN_BIT_COUNT    8U
#define REPLY_1010_MIN_TOGGLE_COUNT 6U
#define REPLY_MIN_AMPLITUDE_MARGIN  50U
#define REPLY_FREQ_DIFF_MIN_HZ      100U
#define REPLY_FREQ_TOLERANCE_HZ     30U

/* PRE_LINKED 参数 */
#define PRE_QUIET_OBSERVE_MAX_MS 4000U
#define PRE_REPLY_DURATION_MS    2000U
#define PRE_ACK_TIMEOUT_MS       5000U
#define PRE_ACK_MAX_RETRY        1U
#define PRE_BFSK_BIT_PERIOD_MS   40U

typedef enum {
    PRE_QUIET_OBSERVE = 0,
    PRE_SELF_REPLY    = 1,
    PRE_WAIT_ACK      = 2,
} pre_linked_phase_t;

/* SCAN_WAIT_REPLY 子阶段 */
typedef enum {
    SCAN_WAIT_1010 = 0,
    SCAN_ACK_SENT  = 1,
} scan_wait_reply_phase_t;

/* 节点全局上下文 */
typedef struct {
    node_state_t       state;
    uint32_t           state_enter_tick;

    node_xact_flags_t  flags;
    uint8_t            pending_buffer[64];
    uint16_t           pending_len;

    node_freq_pair_t   sweep_result;
    uint16_t           sweep_amp_table[SWEEP_FREQ_COUNT];
    uint32_t           sweep_start_tick;
    uint32_t           sweep_timeout_tick;

    uint8_t            scan_freq_index;
    uint32_t           scan_freq_next_tick;
    
    uint32_t           scan_wait_reply_start_tick;
    scan_wait_reply_phase_t scan_wait_reply_phase;
    uint32_t           scan_ack_sent_tick;
    uint8_t            scan_ack_retry_count;
    uint32_t           scan_1010_detected_tick;
    uint16_t           reply_last_freq;
    uint16_t           reply_detected_f0;
    uint16_t           reply_detected_f1;
    uint8_t            reply_toggle_count;

    node_freq_pair_t   uplink_freq;
    node_freq_pair_t   downlink_freq;
    uint8_t            active_link_is_uplink;
    uint16_t           noise_floor;
    uint8_t            hard_id;

    uint32_t           quiet_start_tick;

    uint32_t           pre_linked_last_check_tick;
    pre_linked_phase_t pre_linked_phase;
    uint32_t           pre_phase_enter_tick;
    uint8_t            self_reply_cur_is_f0;
    uint32_t           self_reply_next_bit_tick;
    uint8_t            pre_ack_retry_count;
} app_node_ctx_t;

/* 公共 API */

void app_node_fsm_init(uint8_t hard_id);
void app_node_fsm_run(uint32_t now_tick);

node_state_t app_node_fsm_get_state(void);
bool app_node_fsm_is_clean(void);
bool app_node_fsm_is_uplink_locked(void);
bool app_node_fsm_is_downlink_locked(void);
void app_node_fsm_get_uplink_freq(uint16_t *f0_hz, uint16_t *f1_hz);
void app_node_fsm_get_downlink_freq(uint16_t *f0_hz, uint16_t *f1_hz);
const app_node_ctx_t *app_node_fsm_get_ctx(void);

node_role_t app_node_fsm_get_role(void);
bool app_node_fsm_has_uplink(void);
bool app_node_fsm_has_downlink(void);

/* 事件注入 API */

void app_node_fsm_on_sweep_detected(uint16_t f_hz, uint16_t amplitude, uint32_t now_tick);
void app_node_fsm_on_sweep_complete(uint8_t valid_freq_count);
void app_node_fsm_on_ack_received(uint16_t f0, uint16_t f1);

void app_node_fsm_flag_set_rx_up(void);
void app_node_fsm_flag_set_fwd_dn(void);
void app_node_fsm_flag_set_rx_dn(void);
void app_node_fsm_flag_set_fwd_up(void);

void app_node_fsm_flag_clear_rx_up(void);
void app_node_fsm_flag_clear_fwd_dn(void);
void app_node_fsm_flag_clear_rx_dn(void);
void app_node_fsm_flag_clear_fwd_up(void);

void app_node_fsm_on_quiet_timeout(uint32_t now_tick);
void app_node_fsm_on_fault(uint32_t fault_code);
void app_node_fsm_on_wdt_timeout(void);
void app_node_fsm_on_scan_order(uint8_t scan_direction);

#ifdef __cplusplus
}
#endif

#endif /* APP_NODE_FSM_H */
