/**
 * @file    app_main.h
 * @brief   应用入口 - 井上节点
 * @note    系统初始化 → 创建 FreeRTOS 任务 → 启动调度器
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 节点地址定义 */
#define NODE_ADDR_GROUND    0x01   /* 井上节点地址 */

/* 系统状态 */
typedef enum {
    SYS_STATE_INIT,       /* 上电初始化 */
    SYS_STATE_SCAN,       /* 扫频中 */
    SYS_STATE_LINKED,     /* 链路建立 */
    SYS_STATE_WORK,       /* 正常工作 */
    SYS_STATE_SLEEP,      /* 低功耗休眠 */
    SYS_STATE_ERROR       /* 异常 */
} sys_state_t;

void app_main_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
