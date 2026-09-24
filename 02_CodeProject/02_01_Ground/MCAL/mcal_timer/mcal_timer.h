/**
 * @file    mcal_timer.h
 * @brief   定时器驱动 - MCAL 层
 * @note    提供系统 Tick、微秒延时、硬件超时
 */

#ifndef MCAL_TIMER_H
#define MCAL_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 宏定义 ========== */
#define TIMER_ID_1    1   /* 用于 Goertzel 采样触发 */
#define TIMER_ID_2    2   /* 用于协议超时 */
#define TIMER_ID_3    3   /* 用于 OneWire 精确延时（DS18B20） */

/* ========== 函数声明 ========== */

/**
 * @brief   定时器初始化
 */
void mcal_timer_init(void);

/**
 * @brief   微秒延时（阻塞）
 * @param   us   延时微秒数
 * @note    DS18B20 时序依赖此函数精度
 */
void mcal_timer_delay_us(uint32_t us);

/**
 * @brief   毫秒延时（阻塞）
 * @param   ms   延时毫秒数
 */
void mcal_timer_delay_ms(uint32_t ms);

/**
 * @brief   启动单次定时器中断
 * @param   id        定时器 ID
 * @param   period_us 定时周期（微秒）
 * @note    定时到达后触发回调，不自动重载
 */
void mcal_timer_start_once(uint8_t id, uint32_t period_us);

/**
 * @brief   启动周期定时器
 * @param   id        定时器 ID
 * @param   period_us 定时周期（微秒）
 */
void mcal_timer_start_periodic(uint8_t id, uint32_t period_us);

/**
 * @brief   停止定时器
 * @param   id   定时器 ID
 */
void mcal_timer_stop(uint8_t id);

/* ========== 回调 ========== */
__weak void mcal_timer_callback(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_TIMER_H */
