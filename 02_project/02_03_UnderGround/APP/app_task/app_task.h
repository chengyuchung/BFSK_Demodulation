/**
 * @file    app_task.h
 * @brief   FreeRTOS 任务管理 - APP 层（井下节点）
 */

#ifndef APP_TASK_H
#define APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define TASK_PRIORITY_HIGH     3
#define TASK_PRIORITY_NORMAL   2
#define TASK_PRIORITY_LOW      1

#define TASK_STACK_SIZE_SMALL  256
#define TASK_STACK_SIZE_MEDIUM 512
#define TASK_STACK_SIZE_LARGE  1024

#define TASK_NAME_DEMOD        "TaskDemod"
#define TASK_NAME_COMM         "TaskComm"
#define TASK_NAME_SENSOR       "TaskSensor"
#define TASK_NAME_APP          "TaskApp"
#define TASK_NAME_PRINT        "TaskPrint"

void app_task_create_all(void);
void app_task_delete_all(void);
void app_task_start_scheduler(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_H */
