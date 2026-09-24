/**
 * @file    bsw_scheduler.h
 * @brief   定时调度 - BSW 层
 * @note    FreeRTOS Task 封装，提供周期性执行和一次性定时机制
 */

#ifndef BSW_SCHEDULER_H
#define BSW_SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 类型定义 ========== */
typedef void (*bsw_scheduler_cb_t)(void *arg);

/* 调度任务描述 */
typedef struct {
    const char    *name;
    uint32_t       period_ms;   /* 周期（ms），0 表示单次 */
    bsw_scheduler_cb_t callback;
    void          *arg;
    uint8_t        priority;
} bsw_scheduler_task_t;

/* ========== 函数声明 ========== */

/**
 * @brief   调度器初始化
 */
void bsw_scheduler_init(void);

/**
 * @brief   注册一个任务（创建对应 FreeRTOS Task）
 * @param   task   任务描述
 * @retval  0=成功 其他=失败
 */
int bsw_scheduler_register(const bsw_scheduler_task_t *task);

/**
 * @brief   设置一次性定时器回调
 * @param   delay_ms   延时（ms）
 * @param   callback   回调函数
 * @param   arg        回调参数
 */
void bsw_scheduler_set_timer(uint32_t delay_ms, bsw_scheduler_cb_t callback, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* BSW_SCHEDULER_H */
