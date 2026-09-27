/**
 * @file    bsw_bfsk_demod.h
 * @brief   BFSK 解调器 - BSW 层
 *
 * @note    与 bsw_ad9833（信号发生）配对，构成井下节点的双向通信：
 *              bsw_ad9833      ── 上行回发 / 自载波回显
 *              bsw_bfsk_demod  ── 下行 ACK 接收 / 后续上行数据帧
 *
 * 解调链路：
 *   1. TIM1 输入捕获（mcal_timer_ic_*）记录信号相邻跳变时刻
 *   2. 跳变间隔 Δt → 频率 → BFSK bit（Δt≈1/f0 → 0；Δt≈1/f1 → 1）
 *   3. bit 流累积成帧（帧头 + 数据 + CRC）
 *   4. 解析结果回调给 app_node_fsm
 *
 * 频率对来源：
 *   - 从 s_ctx.cur_freq (app_node_fsm) 注入 f0_hz / f1_hz
 *   - 与 bsw_ad9833 已写入的 FREQ0 / FREQ1 对称
 *
 * @dependency  mcal_timer    (TIM1 输入捕获硬件抽象)
 *              app_node_fsm  (注入 cur_freq + 消费解调结果，事件回调)
 *
 * @usage
 *   bsw_bfsk_demod_init();
 *   // ...在 FSM 进入 PHASE3 时调用：
 *   bsw_bfsk_demod_start(s_ctx.cur_freq.f0_hz, s_ctx.cur_freq.f1_hz,
 *                        timeout_ms);
 *   // 主循环中查询状态：
 *   if (bsw_bfsk_demod_get_state() == BSW_BFSK_DEMOD_DONE_OK) { ... }
 */

#ifndef BSW_BFSK_DEMOD_H
#define BSW_BFSK_DEMOD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========== 解调器状态机 ========== */
typedef enum {
    BSW_BFSK_DEMOD_IDLE          = 0,   /* 未启动 / 已停止 */
    BSW_BFSK_DEMOD_RUNNING       = 1,   /* 监听中，累积 bit 流 */
    BSW_BFSK_DEMOD_DONE_OK       = 2,   /* 帧解析成功，结果可用 */
    BSW_BFSK_DEMOD_DONE_TIMEOUT  = 3,   /* 超时未收到完整帧 */
    BSW_BFSK_DEMOD_DONE_ERR      = 4,   /* CRC 错 / 帧格式错 */
} bsw_bfsk_demod_state_t;

/* ========== 返回码 ========== */
typedef enum {
    BSW_BFSK_DEMOD_OK        =  0,
    BSW_BFSK_DEMOD_ERR_PARAM = -1,   /* 频率参数非法 / 状态不允许 */
    BSW_BFSK_DEMOD_ERR_STATE = -2,   /* 未初始化就调用 / 重复 start */
    BSW_BFSK_DEMOD_ERR_HW    = -3,   /* mcal_timer_ic 启动失败（占位） */
} bsw_bfsk_demod_ret_t;

/* ========== 解调结果（帧结构待定） ========== */
/* TODO: 根据 ACK 帧格式最终确定字段 */
typedef struct {
    uint16_t f0_hz;            /* 当时的低频点（用于核对） */
    uint16_t f1_hz;            /* 时的高频点 */
    uint8_t  bit_count;        /* 实际解出的 bit 数 */
    uint8_t  bit_buf[16];      /* bit 流（占位，待帧格式确定） */
} bsw_bfsk_demod_result_t;

/* ========== 函数声明 ========== */

/**
 * @brief   初始化 BFSK 解调器
 * @note    1) 注册 TIM1 IC 跳变回调到 mcal_timer
 *          2) 内部状态机置 IDLE
 *          3) 不启动硬件采样
 * @retval  BSW_BFSK_DEMOD_OK
 */
bsw_bfsk_demod_ret_t bsw_bfsk_demod_init(void);

/**
 * @brief   启动一次 ACK 帧的解调监听
 * @param   f0_hz       当前锁定低频点（Hz）
 * @param   f1_hz       当前锁定高频点（Hz）
 * @param   timeout_ms  监听超时（ms，协议 §4.2.3 给出上限）
 * @retval  BSW_BFSK_DEMOD_OK / ERR_PARAM / ERR_STATE
 *
 * @note    1) 启用 TIM1 IC（mcal_timer_ic_start）
 *          2) 清空内部 bit 缓冲
 *          3) 启动超时计时器
 *          4) 状态置 RUNNING
 */
bsw_bfsk_demod_ret_t bsw_bfsk_demod_start(uint16_t f0_hz,
                                         uint16_t f1_hz,
                                         uint32_t timeout_ms);

/**
 * @brief   主动停止解调（状态机切走 / 复位 / 调试时用）
 * @note    1) 关闭 TIM1 IC
 *          2) 清空缓冲
 *          3) 状态置 IDLE
 */
bsw_bfsk_demod_ret_t bsw_bfsk_demod_stop(void);

/**
 * @brief   主循环钩子：检查超时 / 处理异步事件
 * @param   now_tick    HAL_GetTick() 或 OS 时钟
 * @note    需在 app_task 10ms 节拍器里调用
 */
void bsw_bfsk_demod_run(uint32_t now_tick);

/**
 * @brief   查询解调器状态
 */
bsw_bfsk_demod_state_t bsw_bfsk_demod_get_state(void);

/**
 * @brief   取出最近一次 DONE_OK 的解调结果（消费一次后清空）
 * @param[out] out   目标缓冲区
 * @retval  BSW_BFSK_DEMOD_OK           已填充
 *          BSW_BFSK_DEMOD_ERR_PARAM    out 为 NULL
 *          BSW_BFSK_DEMOD_ERR_STATE    当前无有效结果
 */
bsw_bfsk_demod_ret_t bsw_bfsk_demod_take_result(bsw_bfsk_demod_result_t *out);

/* ========== 内部回调（mcal_timer 调用，不要在应用层直接调） ========== */

/**
 * @brief   TIM1 IC 中断回调：每次信号跳变触发
 * @param   tick_us    跳变发生的时刻（μs，由 TIM1 CCR 换算）
 * @note    由 mcal_timer_ic_register_callback() 注册
 *          本函数决定 Δt → bit → 帧累积
 */
void bsw_bfsk_demod_on_ic_edge(uint32_t tick_us);

#ifdef __cplusplus
}
#endif

#endif /* BSW_BFSK_DEMOD_H */
