/**
 * @file    app_main.h - 井下节点 APP 层入口
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define NODE_ADDR_UNDERGROUND  0x03

/**
 * @brief   系统状态机枚举（APP 层全局）
 */
typedef enum {
    SYS_STATE_INIT = 0,
    SYS_STATE_SCAN,
    SYS_STATE_LINKED,
    SYS_STATE_WORK,
    SYS_STATE_SLEEP,
    SYS_STATE_ERROR
} sys_state_t;

/**
 * @brief   APP 层初始化入口
 * @note    按层级调用：MCAL → BSW → APP → FreeRTOS 任务
 */
void app_main_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
