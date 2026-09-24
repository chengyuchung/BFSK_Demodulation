/* bsw_scheduler.c */
#include "bsw_scheduler.h"

void bsw_scheduler_init(void)
{
    /* TODO: FreeRTOS 调度器初始化 */
}

int bsw_scheduler_register(const bsw_scheduler_task_t *task)
{
    (void)task;
    /* TODO: xTaskCreate() 创建对应 FreeRTOS Task */
    return 0;
}

void bsw_scheduler_set_timer(uint32_t delay_ms, bsw_scheduler_cb_t callback, void *arg)
{
    (void)delay_ms;(void)callback;(void)arg;
    /* TODO: xTimerCreate + xTimerStart */
}
