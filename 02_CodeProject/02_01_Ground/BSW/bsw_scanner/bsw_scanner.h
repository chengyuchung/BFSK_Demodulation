/**
 * @file    bsw_scanner.h
 * @brief   扫频逻辑 - BSW 层
 * @note    19 频点遍历 + 监听-发射错相轮换
 *
 * @dependency  bsw_bfsk
 */

#ifndef BSW_SCANNER_H
#define BSW_SCANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 宏定义 ========== */
#define SCAN_FREQ_MIN     100000U   /* 100kHz */
#define SCAN_FREQ_MAX     1000000U  /* 1MHz */
#define SCAN_FREQ_STEP    50000U    /* 步进 50kHz */
#define SCAN_FREQ_COUNT   19        /* (1MHz-100kHz)/50kHz + 1 = 19 */

/* ========== 类型定义 ========== */

/* 扫频状态 */
typedef enum {
    SCAN_STATE_IDLE,       /* 空闲 */
    SCAN_STATE_LISTEN,     /* 监听模式（等待对方扫频） */
    SCAN_STATE_TRANSMIT,   /* 发射模式（发送扫频信号） */
    SCAN_STATE_LOCKED,     /* 锁定成功 */
    SCAN_STATE_FAILED      /* 扫频失败 */
} bsw_scan_state_t;

/* 扫频结果 */
typedef struct {
    uint32_t f0;   /* 锁定后频率 0 */
    uint32_t f1;   /* 锁定后频率 1 */
    uint8_t  peer_addr; /* 对端地址 */
    int      is_valid;
} bsw_scan_result_t;

/* ========== 函数声明 ========== */

/**
 * @brief   初始化扫频模块
 */
void bsw_scanner_init(void);

/**
 * @brief   开始扫频（作为发射方）
 * @note    遍历 19 个频点，每个频点发射一段时间后切换
 */
void bsw_scanner_start_as_transmitter(void);

/**
 * @brief   开始扫频（作为接收方）
 * @note    遍历 19 个频点，每个频点监听一段时间后切换
 */
void bsw_scanner_start_as_receiver(void);

/**
 * @brief   扫频主循环（每次调度调用一次）
 * @retval  SCAN_STATE_LOCKED 时结果在 result 里
 */
bsw_scan_state_t bsw_scanner_tick(bsw_scan_result_t *result);

/**
 * @brief   停止扫频
 */
void bsw_scanner_stop(void);

/**
 * @brief   获取当前扫频状态
 */
bsw_scan_state_t bsw_scanner_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_SCANNER_H */
