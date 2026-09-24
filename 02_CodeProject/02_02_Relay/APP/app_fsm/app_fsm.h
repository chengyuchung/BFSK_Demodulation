/* app_fsm.h - 中继节点 */
#ifndef APP_FSM_H
#define APP_FSM_H
#ifdef __cplusplus
extern "C" {
#endif
#include "app_main.h"
typedef enum {
    EV_SCAN_START, EV_SCAN_DONE, EV_LINK_UP, EV_LINK_DOWN,
    EV_FRAME_RECV, EV_TIMEOUT, EV_SLEEP_CMD, EV_WAKEUP, EV_ERROR
} fsm_event_t;
void app_fsm_init(void);
void app_fsm_dispatch(fsm_event_t ev);
sys_state_t app_fsm_get_state(void);
#ifdef __cplusplus
}
#endif
#endif /* APP_FSM_H */
