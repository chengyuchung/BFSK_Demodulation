/**
 * @file    app_node_fsm.h
 * @brief   井下节点状态机模块
 *
 * 定义节点状态枚举、上下文结构体及对外接口。
 *
 * 状态转移遵循 扫频启动协议.md：
 *   - §一  扫频启动识别（三重物理特征）
 *   - §二  扫频结束判定（850 ms 硬超时 + 尾部静默检测）
 *   - §五  断链角色仲裁（4 标志位事务上下文）
 *   - §六  全生命周期休眠逻辑（清洁态鉴别）
 */

#ifndef APP_NODE_FSM_H
#define APP_NODE_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 标准库 ========== */
#include <stdint.h>
#include <stdbool.h>

/* ========== 子模块头文件 ========== */
#include "mcal_timer.h"   /* mcal_timer_base_start/stop 依赖 */

/* ========== 节点状态枚举 ========== */

typedef enum {
    NODE_BOOT = 0,          /**< 启动中：时钟/外设初始化，4 标志位全 0 */
    NODE_SCAN_LISTEN,        /**< 扫频监听（被动）：ADC 持续 10 kHz，等上级 19 频点扫频 */
    NODE_SCAN,               /**< 扫频进行中：命中第一个频点，填 19 格成绩单，启动 850 ms 会话定时器 */
    NODE_PRE_LINKED,         /**< 预链接（协议 §4.2）：已选出 (f0,f1)，等待通知地面机 + 等 ACK */
    NODE_LINKED,             /**< 工作模式：与上级锁定频对 (f0, f1)，可收发 BFSK 帧 */
    NODE_SLEEP,              /**< 间歇休眠：清洁态（4 标志位全 0）切入，900 ms 睡 / 100 ms 听 */
    NODE_FAULT               /**< 不可恢复故障：硬件异常或看门狗失败 */
} node_state_t;

/* ========== 4 标志位事务上下文（位域紧凑） ========== */

typedef enum {
    XACT_RX_UP_OK   = 0,   /**< ① 从上接收成功：已收到上级命令 */
    XACT_FWD_DN_OK  = 1,   /**< ② 向下转发成功：已发给下级并收到下行 ACK */
    XACT_RX_DN_OK   = 2,   /**< ③ 从下接收成功：已收到下级回传数据 */
    XACT_FWD_UP_OK  = 3    /**< ④ 向上转发成功：已发给上级并收到上行 ACK */
} node_xact_bit_t;

/**
 * @brief   4 标志位（协议 §5.1）
 * @note    任一非默认 = 事务挂起态，绝对禁止进入间歇休眠
 */
typedef struct {
    uint8_t flag_rx_up_ok  : 1;   /**< ① 从上接收成功 */
    uint8_t flag_fwd_dn_ok : 1;   /**< ② 向下转发成功 */
    uint8_t flag_rx_dn_ok  : 1;   /**< ③ 从下接收成功 */
    uint8_t flag_fwd_up_ok : 1;   /**< ④ 向上转发成功 */
} node_xact_flags_t;

/* ========== 工作频对 ========== */

typedef struct {
    uint16_t f0_hz;   /**< 低频点 Hz */
    uint16_t f1_hz;   /**< 高频点 Hz */
} node_freq_pair_t;

/* ========== 扫频成绩单（协议 §三） ========== */

/**
 * 19 个候选频点：125 Hz～1025 Hz，步进 50 Hz
 * 数组索引 0 对应 125 Hz，18 对应 1025 Hz
 */
#define SWEEP_FREQ_COUNT  19
#define SWEEP_FREQ_START  125
#define SWEEP_FREQ_STEP   50
#define SWEEP_FREQ_TOLERANCE_HZ  15   /**< 标称频点就近映射容差 ±15 Hz */

/* ========== 频对决选参数（协议 §三） ========== */
#define SWEEP_MIN_FREQ_GAP_HZ    150u   /**< 协议 §三：f1 - f0 最小间距 (Hz) */
#define SWEEP_VALID_AMP_DELTA    50u    /**< 协议 §三：amp > noise_floor + 该值 才算有效信号
                                        //   *   用于把 amp_table 里的"信号"和"噪声"区分开：
                                        //   *   没有这个门限，19 格噪声密集抖动时会被误认为 19 个有效信号，
                                        //   *   max+second_max 可能选出"噪声最强两格"导致 1010 回发失败 */
#define SWEEP_DEFAULT_NOISE_FLOOR 100u  /**< 默认底噪估值（amp 单位），noise_floor 未在线测量时使用 */

/* ========== PRE_LINKED 子状态机参数（协议 §4.2） ==========
 *
 * PRE_LINKED 三阶段：
 *   PHASE1 QUIET_OBSERVE  等待本节点响应槽点 = my_id × BSW_NODE_REPLY_STEP_MS（200 ms）
 *                         ├─ 每 40 ms 用 BSW ADC ringbuf 算 RMS²
 *                         │   超 BSW_ADC_RINGBUF_SIGNAL_THRESHOLD → 别人抢在我前面回发
 *                         │      → 退让回 SCAN_LISTEN（等下一轮扫频重新排槽）
 *                         └─ 走到自己槽点（hard_id × 200 ms）还没人抢 → 进 PHASE2
 *   PHASE2 SELF_REPLY     在 (f0, f1) 上交替 101010... 持续 PRE_REPLY_DURATION_MS（2 s）
 *   PHASE3 WAIT_ACK       等井上 ACK（暂留空，等自载波解调 API 接入）
 *
 * 槽点时序常量由 BSW/bsw_node_id/bsw_node_id.h 提供：
 *   BSW_NODE_REPLY_STEP_MS = 200 ms
 *   bsw_node_id_get_reply_delay_ms() = my_id × 200 ms（本节点槽偏移）
 */
#define PRE_QUIET_OBSERVE_MAX_MS  4000U
#define PRE_REPLY_DURATION_MS    2000U
#define PRE_ACK_TIMEOUT_MS       2000U
#define PRE_BFSK_BIT_PERIOD_MS   40U

typedef enum {
    PRE_QUIET_OBSERVE = 0,
    PRE_SELF_REPLY    = 1,
    PRE_WAIT_ACK      = 2,
} pre_linked_phase_t;

/* ========== 节点全局上下文 ========== */

/**
 * @brief   节点全局上下文（64 字节紧凑设计）
 * @note    供状态机内部决策，不直接暴露给其他模块
 */
typedef struct {
    node_state_t       state;               /**< 当前状态 */
    uint32_t           state_enter_tick;    /**< 进入当前状态的时刻（HAL_GetTick） */

    /* ---- 事务上下文（协议 §5）---- */
    node_xact_flags_t  flags;               /**< 4 标志位 */
    uint8_t            pending_buffer[64];  /**< 未完成命令/数据的暂存区 */
    uint16_t           pending_len;         /**< 当前暂存有效字节数 */

    /* ---- 扫频数据（协议 §三）---- */
    node_freq_pair_t   sweep_result;       /**< 扫频决选后的工作频对（f0, f1） */
    uint16_t           sweep_amp_table[SWEEP_FREQ_COUNT];  /**< 19 格幅值成绩单 */
    uint32_t           sweep_start_tick;    /**< 命中第一个频点的时刻 */
    uint32_t           sweep_timeout_tick; /**< 850 ms 会话硬超时到期时刻 */

    /* ---- 工作态参数 ---- */
    node_freq_pair_t   cur_freq;            /**< 当前锁定的工作频对，f0=0 表示未锁定 */
    uint16_t           noise_floor;         /**< IDLE 期背景底噪（协议门槛一） */
    uint8_t            hard_id;             /**< 节点硬件 ID（井下底节 = 3） */

    /* ---- 休眠鉴别（协议 §6）---- */
    uint32_t           quiet_start_tick;    /**< 总线进入静默的时刻（清洁态鉴别） */

    /* ---- PRE_LINKED 子状态 ---- */
    uint32_t           pre_linked_last_check_tick;
    pre_linked_phase_t pre_linked_phase;
    uint32_t           pre_phase_enter_tick;
    uint8_t            self_reply_cur_is_f0;
    uint32_t           self_reply_next_bit_tick;
} app_node_ctx_t;

/* ========== 公共 API ========== */

/**
 * @brief   初始化节点状态机
 * @param   hard_id   节点硬件 ID（井下底节填 3）
 */
void app_node_fsm_init(uint8_t hard_id);

/**
 * @brief   状态机主循环钩（每 10 ms 由 app_task 调用一次）
 * @param   now_tick  当前 HAL_GetTick()
 */
void app_node_fsm_run(uint32_t now_tick);

/**
 * @brief   状态查询
 */
node_state_t  app_node_fsm_get_state(void);
bool           app_node_fsm_is_clean(void);        /**< 4 标志位全 0 = 清洁态，可进休眠 */
bool           app_node_fsm_is_freq_locked(void);   /**< f0 ≠ 0 */
const app_node_ctx_t *app_node_fsm_get_ctx(void);  /**< 调试/业务模块只读上下文 */

/* ========== 事件注入 API（由业务模块调用） ========== */

/** 扫频监听态的扫频检测算法（10 ms 滑动窗口）注入一次解算结果
 * @param   f_hz       本次窗口解算出的瞬时频率
 * @param   amplitude  本次窗口解算出的峰峰值
 * @param   now_tick   注入时刻（HAL_GetTick），FSM 内部用它启动 850 ms 倒计时
 * @note    调用时机：每 10 ms 由扫频检测算法调一次，前提是三重特征校验通过。
 *          FSM 内部把 f_hz 就近映射到 amp_table 19 格中的某一格，按
 *          协议 §三"只记录不淘汰"规则写入。
 *          SCAN_LISTEN 期间会被多次调用；FSM 不会因为这次调用退出 SCAN_LISTEN。 */
void app_node_fsm_on_sweep_detected(uint16_t f_hz, uint16_t amplitude, uint32_t now_tick);

/** 扫频窗口结算（850 ms 超时或尾部静默提前收敛） */
void app_node_fsm_on_sweep_complete(uint8_t valid_freq_count);

/** 上级用 (f0, f1) 下发 ACK，链路锁定 */
void app_node_fsm_on_ack_received(uint16_t f0, uint16_t f1);

/** 4 标志位置位 */
void app_node_fsm_flag_set(node_xact_bit_t bit);

/** 4 标志位清除 */
void app_node_fsm_flag_clear(node_xact_bit_t bit);

/** 总线静默超时（协议 §6.2 降级休眠判断） */
void app_node_fsm_on_quiet_timeout(uint32_t now_tick);

/** 硬件/协议不可恢复故障 */
void app_node_fsm_on_fault(uint32_t fault_code);

/** 看门狗超时 */
void app_node_fsm_on_wdt_timeout(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_NODE_FSM_H */
