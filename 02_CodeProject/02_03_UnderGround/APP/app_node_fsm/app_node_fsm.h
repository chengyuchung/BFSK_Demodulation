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
    NODE_SCAN,               /**< 扫频发送（主动）：上级节点发送 19 个频点 × 40ms */
    NODE_SCAN_WAIT_REPLY,    /**< 扫频等待回应：上级节点监听下级的 1010 波形，解析频对 */
    NODE_PRE_LINKED,         /**< 预链接（协议 §4.2）：下级节点已选出 (f0,f1)，发送 1010，等 ACK */
    NODE_LINKED,             /**< 工作模式：与上/下级锁定频对 (f0, f1)，可收发 BFSK 帧 */
    NODE_SLEEP,              /**< 间歇休眠：清洁态（4 标志位全 0）切入，900 ms 睡 / 100 ms 听 */
    NODE_FAULT               /**< 不可恢复故障：硬件异常或看门狗失败 */
} node_state_t;

/* ========== 4 标志位事务上下文（位域紧凑） ========== */

/**
 * @brief   4 标志位
 * @note    任一非默认 = 事务挂起态，绝对禁止进入间歇休眠
 *          同时作为链路健康度评估（页面状态切换依据）
 */
typedef struct {
    uint8_t flag_rx_up_ok  : 1;   /**< 从上接收成功 */
    uint8_t flag_fwd_dn_ok : 1;   /**< 向下转发成功 */
    uint8_t flag_rx_dn_ok  : 1;   /**< 从下接收成功 */
    uint8_t flag_fwd_up_ok : 1;   /**< 向上转发成功 */
} node_xact_flags_t;

/* ========== 工作频对 ========== */

typedef struct {
    uint16_t f0_hz;   /**< 低频点 Hz */
    uint16_t f1_hz;   /**< 高频点 Hz */
} node_freq_pair_t;

/* ========== 扫频成绩单 ========== */

/**
 * 19 个候选频点：125 Hz～1025 Hz，步进 50 Hz
 * 数组索引 0 对应 125 Hz，18 对应 1025 Hz
 */
#define SWEEP_FREQ_COUNT  19
#define SWEEP_FREQ_START  125
#define SWEEP_FREQ_STEP   50
#define SWEEP_FREQ_TOLERANCE_HZ  15   /**< 标称频点就近映射容差 ±15 Hz */
#define SWEEP_FREQ_INDEX_INVALID 0xFFU /**< 无效频点索引（落在容差外/空白带） */

/* ========== 频对决选参数 ========== */
#define SWEEP_MIN_FREQ_GAP_HZ    150u   /**< f1 - f0 最小间距 (Hz) */
#define SWEEP_VALID_AMP_DELTA    50u    /**< amp > noise_floor + 该值 才算有效信号
                                        //   *   用于把 amp_table 里的"信号"和"噪声"区分开：
                                        //   *   没有这个门限，19 格噪声密集抖动时会被误认为 19 个有效信号，
                                        //   *   max+second_max 可能选出"噪声最强两格"导致 1010 回发失败 */
#define SWEEP_DEFAULT_NOISE_FLOOR 100u  /**< 默认底噪估值（amp 单位），noise_floor 未在线测量时使用 */
#define SWEEP_MIN_VALID_FREQ_CNT  2u    /**< 频对决选最少需要的有效频点数 */
#define SWEEP_SESSION_TIMEOUT_MS  850U  /**< 扫频会话硬超时（毫秒） */

/* ========== 时间常量 ========== */
#define BUS_QUIET_TIMEOUT_MS      3000U /**< 总线静默判定超时（降级休眠） */
#define ADC_CHECK_PERIOD_MS       40U   /**< PRE_LINKED PHASE1 ADC 能量检测周期 */
#define SCAN_FREQ_DURATION_MS     40U   /**< SCAN 状态每个频点持续时间（毫秒） */
#define SCAN_WAIT_REPLY_TIMEOUT_MS 5000U  /**< SCAN_WAIT_REPLY 状态等待下级 1010 回应超时（毫秒） */
#define SCAN_ACK_CONFIRM_TIMEOUT_MS 6000U /**< SCAN_WAIT_REPLY ACK 确认等待超时（毫秒）
                                            // *  注意：必须 > PRE_ACK_TIMEOUT_MS (5s)
                                            // *  原因：被扫方等待 ACK 5s 超时后才会重发 1010
                                            // *       扫频方需要等待 > 5s 才能确认"对方收到了 ACK"
                                            // *       如果 < 5s 就进入 LINKED，可能漏掉被扫方的重发 1010 */
#define SCAN_ACK_MAX_RETRY        2U     /**< ACK 最大重发次数 */

/* ========== 节点角色定义 ========== */

/**
 * @brief 节点角色类型
 * 根据硬件 ID（hard_id）判断节点在通信链路中的角色
 * 使用 bsw_node_id.h 中的地址定义
 */
typedef enum {
    NODE_ROLE_SURFACE    = 0,  /**< 井上节点（BSW_NODE_ADDR_GROUND = 0）：只有下行链路 */
    NODE_ROLE_RELAY      = 1,  /**< 中继节点（1~14）：既有上行也有下行 */
    NODE_ROLE_UNDERGROUND = 2   /**< 井下节点（BSW_NODE_ADDR_UNDERGROUND = 0xF）：只有上行链路 */
} node_role_t;

/* ========== 链路方向常量 ========== */
#define LINK_UPLINK               1U     /**< 活动链路：上级链路（与上级通信） */
#define LINK_DOWNLINK             0U     /**< 活动链路：下级链路（与下级通信） */

/* ========== 标志位状态常量 ========== */
#define FLAG_SET                  1U     /**< 标志位置位 */
#define FLAG_CLEARED              0U     /**< 标志位清除 */

/* ========== 频率状态常量 ========== */
#define FREQ_NOT_LOCKED           0U     /**< 频率未锁定 */

/* ========== 时刻状态常量 ========== */
#define QUIET_NOT_STARTED         0U     /**< 静默检测未开始 */

/* ========== 1010 检测参数 ========== */
#define REPLY_1010_MIN_BIT_COUNT  8U     /**< 1010 模式最少比特数（10101010 = 8 bit） */
#define REPLY_1010_MIN_TOGGLE_COUNT 6U   /**< 1010 模式最少交替次数（f0→f1→f0→f1→f0→f1→f0 = 6 次切换） */
#define REPLY_MIN_AMPLITUDE_MARGIN 50U   /**< 1010 回应最小振幅余量（基于有效信号门槛） */
#define REPLY_FREQ_DIFF_MIN_HZ    100U   /**< f0 与 f1 最小频率差（Hz，避免噪声干扰） */
#define REPLY_FREQ_TOLERANCE_HZ   30U    /**< 频率匹配容差（±Hz） */

/* ========== PRE_LINKED 子状态机参数 ==========
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
#define PRE_ACK_TIMEOUT_MS       5000U   /* PHASE3 等待 ACK 超时 5s */
#define PRE_ACK_MAX_RETRY        1U      /* 最多重试 1 次（超时后重发 1010，再等 5s）*/
#define PRE_BFSK_BIT_PERIOD_MS   40U    /* 40 ms/bit：实用平衡点
                                        //   * f0=125Hz → 5 周期（多数表决，容错2个 = 40%噪声率）
                                        //   * f1=1025Hz → 41 周期（极高稳定性）
                                        //   * 速率 = 25 bps（比 50ms 快 25%）
                                        //   * 2000ms 应答 → 50 bit（充足）
                                        //   * 单帧(200bit) = 8s（优秀）*/

typedef enum {
    PRE_QUIET_OBSERVE = 0,
    PRE_SELF_REPLY    = 1,
    PRE_WAIT_ACK      = 2,
} pre_linked_phase_t;

/* ========== SCAN_WAIT_REPLY 子阶段 ========== */

/**
 * @brief SCAN_WAIT_REPLY 状态子阶段
 * 
 * PHASE1 WAIT_1010: 等待下级的 1010 波形（使用 ADC 频率检测）
 * PHASE2 ACK_SENT:  已发送 ACK，等待确认（使用 BFSK 解调器监听重复的 1010）
 */
typedef enum {
    SCAN_WAIT_1010 = 0,  /**< 等待下级 1010 波形 */
    SCAN_ACK_SENT  = 1,  /**< 已发送 ACK，确认等待中 */
} scan_wait_reply_phase_t;

/* ========== 节点全局上下文 ========== */

/**
 * @brief   节点全局上下文（64 字节紧凑设计）
 * @note    供状态机内部决策，不直接暴露给其他模块
 */
typedef struct {
    node_state_t       state;               /**< 当前状态 */
    uint32_t           state_enter_tick;    /**< 进入当前状态的时刻（HAL_GetTick） */

    /* ---- 事务上下文---- */
    node_xact_flags_t  flags;               /**< 4 标志位 */
    uint8_t            pending_buffer[64];  /**< 未完成命令/数据的暂存区 */
    uint16_t           pending_len;         /**< 当前暂存有效字节数 */

    /* ---- 扫频数据---- */
    node_freq_pair_t   sweep_result;       /**< 扫频决选后的工作频对（f0, f1） */
    uint16_t           sweep_amp_table[SWEEP_FREQ_COUNT];  /**< 19 格幅值成绩单 */
    uint32_t           sweep_start_tick;    /**< 命中第一个频点的时刻 */
    uint32_t           sweep_timeout_tick; /**< 850 ms 会话硬超时到期时刻 */

    /* ---- SCAN 状态主动扫频发送 ---- */
    uint8_t            scan_freq_index;     /**< 当前发送的频点索引（0~18） */
    uint32_t           scan_freq_next_tick; /**< 下一次切换频点的时刻 */
    
    /* ---- SCAN_WAIT_REPLY 状态等待下级回应 ---- */
    uint32_t           scan_wait_reply_start_tick; /**< 进入等待回应状态的时刻 */
    scan_wait_reply_phase_t scan_wait_reply_phase; /**< SCAN_WAIT_REPLY 子阶段 */
    uint32_t           scan_ack_sent_tick;         /**< ACK 发送时刻 */
    uint8_t            scan_ack_retry_count;       /**< ACK 重发次数 */
    uint32_t           scan_1010_detected_tick;    /**< 检测到重复 1010 的时刻（用于等待发送完成）*/
    uint16_t           reply_last_freq;            /**< 上一次检测到的频率 */
    uint16_t           reply_detected_f0;          /**< 检测到的第一个频率 */
    uint16_t           reply_detected_f1;          /**< 检测到的第二个频率 */
    uint8_t            reply_toggle_count;         /**< 交替切换计数（确认是 1010 模式） */

    /* ---- 工作态参数 ---- */
    node_freq_pair_t   uplink_freq;      /**< 上级频点（与上级通信，被扫方建立） */
    node_freq_pair_t   downlink_freq;    /**< 下级频点（与下级通信，扫频方建立） */
    uint8_t            active_link_is_uplink; /**< 当前活动链路：LINK_UPLINK=上级链路，LINK_DOWNLINK=下级链路 */
    uint16_t           noise_floor;      /**< IDLE 期背景底噪（协议门槛一） */
    uint8_t            hard_id;          /**< 节点硬件 ID（井下底节 = 3） */

    /* ---- 休眠鉴别（协议 §6）---- */
    uint32_t           quiet_start_tick;    /**< 总线进入静默的时刻（清洁态鉴别） */

    /* ---- PRE_LINKED 子状态 ---- */
    uint32_t           pre_linked_last_check_tick;
    pre_linked_phase_t pre_linked_phase;
    uint32_t           pre_phase_enter_tick;
    uint8_t            self_reply_cur_is_f0;
    uint32_t           self_reply_next_bit_tick;
    uint8_t            pre_ack_retry_count;         /**< PHASE3 重试计数（最多重试1次）*/
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
/**
 * @brief   判断是否已锁定上级频点
 */
bool app_node_fsm_is_uplink_locked(void);

/**
 * @brief   判断是否已锁定下级频点
 */
bool app_node_fsm_is_downlink_locked(void);

/**
 * @brief   获取上级频点
 */
void app_node_fsm_get_uplink_freq(uint16_t *f0_hz, uint16_t *f1_hz);

/**
 * @brief   获取下级频点
 */
void app_node_fsm_get_downlink_freq(uint16_t *f0_hz, uint16_t *f1_hz);
const app_node_ctx_t *app_node_fsm_get_ctx(void);  /**< 调试/业务模块只读上下文 */

/**
 * @brief   获取节点角色
 * @return  NODE_ROLE_SURFACE / NODE_ROLE_RELAY / NODE_ROLE_UNDERGROUND
 */
node_role_t app_node_fsm_get_role(void);

/**
 * @brief   判断节点是否有上行链路
 * @return  true = 有上级节点（中继或井下）
 *          false = 井上节点，无上级
 */
bool app_node_fsm_has_uplink(void);

/**
 * @brief   判断节点是否有下行链路
 * @return  true = 有下级节点（井上或中继）
 *          false = 井下节点，无下级
 */
bool app_node_fsm_has_downlink(void);

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

/** 4 标志位置位：rx_up=已收到上级命令 */
void app_node_fsm_flag_set_rx_up(void);
/** 4 标志位置位：fwd_dn=已发给下级并收到下行 ACK */
void app_node_fsm_flag_set_fwd_dn(void);
/** 4 标志位置位：rx_dn=已收到下级回传数据 */
void app_node_fsm_flag_set_rx_dn(void);
/** 4 标志位置位：fwd_up=已发给上级并收到上行 ACK */
void app_node_fsm_flag_set_fwd_up(void);

/** 4 标志位清除：rx_up */
void app_node_fsm_flag_clear_rx_up(void);
/** 4 标志位清除：fwd_dn */
void app_node_fsm_flag_clear_fwd_dn(void);
/** 4 标志位清除：rx_dn */
void app_node_fsm_flag_clear_rx_dn(void);
/** 4 标志位清除：fwd_up */
void app_node_fsm_flag_clear_fwd_up(void);

/** 总线静默超时 */
void app_node_fsm_on_quiet_timeout(uint32_t now_tick);

/** 硬件/协议不可恢复故障 */
void app_node_fsm_on_fault(uint32_t fault_code);

/** 看门狗超时 */
void app_node_fsm_on_wdt_timeout(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_NODE_FSM_H */
