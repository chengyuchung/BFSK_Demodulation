/**
 * @file    app_task.h
 * @brief   FreeRTOS 任务管理 - APP 层
 * @note    统一管理所有 FreeRTOS 任务创建、消息队列、信号量
 *
 * @dependency  bsw_scheduler (调度封装)
 */

#ifndef APP_TASK_H
#define APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 宏定义：任务优先级 ========== */
#define TASK_PRIORITY_HIGH     3
#define TASK_PRIORITY_NORMAL   2
#define TASK_PRIORITY_LOW      1

/* ========== 宏定义：任务堆栈深度 ========== */
#define TASK_STACK_SIZE_SMALL  256     /* 传感器采集等轻量任务 */
#define TASK_STACK_SIZE_MEDIUM 512    /* 解调/通信任务 */
#define TASK_STACK_SIZE_LARGE  1024    /* 主任务 */

#define TASK_NAME_DEMOD        "TaskDemod"      /* BFSK 解调任务 */
#define TASK_NAME_COMM         "TaskComm"       /* 通信/扫频任务 */
#define TASK_NAME_SENSOR       "TaskSensor"     /* 传感器采集任务 */
#define TASK_NAME_APP          "TaskApp"        /* 业务逻辑任务 */
#define TASK_NAME_PRINT        "TaskPrint"      /* 调试打印任务 */

/* ========== 函数声明 ========== */

/**
 * @brief   创建所有 FreeRTOS 任务
 * @note    在 FreeRTOS 启动前调用
 */
void app_task_create_all(void);

/**
 * @brief   删除所有任务（进入休眠前调用）
 */
void app_task_delete_all(void);

/**
 * @brief   任务调度启动
 * @note    调用 vTaskStartScheduler()，永不返回
 */
void app_task_start_scheduler(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_H */
