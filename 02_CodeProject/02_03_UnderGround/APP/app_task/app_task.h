/**
 * @file    app_task.h
 * @brief   APP 层 FreeRTOS 任务统一管理
 * @note    所有业务任务的创建都集中在 app_task.c 中
 *          app_main_init() 调一次 app_task_create_all() 即可
 *          osKernelStart() 仍由 main.c 调用
 *
 * 设计要点：
 *   - 任务描述符表（task_desc_t 数组）：以后新增任务只追加一行
 *   - 集中式 osThreadNew 调用，便于统一打日志 / 错误处理
 *   - 不持有任何业务逻辑，只做"注册与创建"
 *   - 任务参数全部走宏定义（优先级 / 栈大小 / tick 周期）
 *     方便后期调参与剪裁
 */

#ifndef APP_TASK_H
#define APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 任务参数（集中宏定义，便于后期修改） ==========
 *
 * 加新任务时，在这里追加一组 "<TASK>_<PARAM>" 宏，然后在
 * app_task.c 用 osThreadDef / osDelay 等引用即可。
 */

/* ---- Sensor 任务：周期性更新井下温压数据 ---- */

/** 任务优先级（CMSIS-RTOS v1 枚举：osPriorityIdle / Low / BelowNormal / Normal / AboveNormal / High / Realtime） */
#ifndef APP_TASK_SENSOR_PRIORITY
#define APP_TASK_SENSOR_PRIORITY      osPriorityNormal
#endif

/** 任务栈大小（单位：字节；CMSIS-RTOS v1 stacksize 参数是 BYTES 不是 WORDS） */
#ifndef APP_TASK_SENSOR_STACK_BYTES
#define APP_TASK_SENSOR_STACK_BYTES   384U    /* 96 word，对齐 FreeRTOS 栈粒度 */
                                            
#endif

/** Sensor 任务调度 tick（任务每多少 ms 被调度一次）
 *  真实采样节奏由 app_sensor.c 内部按 APP_SENSOR_SAMPLE_PERIOD_MS 控制，
 *  此处只需要保证 task 调用频率 ≪ 采样周期即可（100ms 是合适粒度）。 */
#ifndef APP_TASK_SENSOR_TICK_MS
#define APP_TASK_SENSOR_TICK_MS       100U
#endif


/**
 * @brief   创建 APP 层所有 FreeRTOS 任务
 * @note    必须先完成以下前置初始化：
 *          - HAL_Init()
 *          - 所有 MCAL 初始化（app_main_init 内已包含）
 *          - 所有 BSW 初始化（app_main_init 内已包含）
 *
 *          本函数只创建任务，不启动调度器。
 *          调度器由 main.c 通过 osKernelStart() 启动。
 */
void app_task_create_all(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_H */
