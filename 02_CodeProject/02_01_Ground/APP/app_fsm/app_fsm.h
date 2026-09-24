/**
 * @file    app_fsm.h
 * @brief   系统状态机 - APP 层（井上节点）
 */

#ifndef APP_FSM_H
#define APP_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_main.h"

/* ========== 状态机事件 ========== */
typedef enum {
    EV_SCAN_START,         /* 开始扫频 */
    EV_SCAN_DONE,         /* 扫频完成 */
    EV_LINK_UP,           /* 链路建立 */
    EV_LINK_DOWN,         /* 链路断开 */
    EV_FRAME_RECV,        /* 收到帧 */
    EV_TIMEOUT,           /* 超时 */
    EV_SLEEP_CMD,        /* 进入休眠 */
    EV_WAKEUP,           /* 唤醒 */
    EV_ERROR             /* 错误 */
} fsm_event_t;

/* ========== 函数声明 ========== */

/**
 * @brief   状态机初始化
 */
void app_fsm_init(void);

/**
 * @brief   状态机主循环（每次事件调用一次）
 * @param   ev   事件
 */
void app_fsm_dispatch(fsm_event_t ev);

/**
 * @brief   获取当前状态
 */
sys_state_t app_fsm_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FSM_H */
