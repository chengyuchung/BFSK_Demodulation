/* app_fsm.c */
#include "app_fsm.h"

static sys_state_t s_state = SYS_STATE_INIT;

void app_fsm_init(void)
{
    s_state = SYS_STATE_INIT;
}

sys_state_t app_fsm_get_state(void)
{
    return s_state;
}

void app_fsm_dispatch(fsm_event_t ev)
{
    switch (s_state) {
        case SYS_STATE_INIT:
            if (ev == EV_SCAN_START) {
                s_state = SYS_STATE_SCAN;
                /* TODO: 启动扫频 */
            }
            break;

        case SYS_STATE_SCAN:
            if (ev == EV_SCAN_DONE) {
                s_state = SYS_STATE_LINKED;
            } else if (ev == EV_ERROR) {
                s_state = SYS_STATE_ERROR;
            }
            break;

        case SYS_STATE_LINKED:
            if (ev == EV_LINK_UP) {
                s_state = SYS_STATE_WORK;
            }
            break;

        case SYS_STATE_WORK:
            if (ev == EV_LINK_DOWN) {
                s_state = SYS_STATE_SCAN;
            } else if (ev == EV_SLEEP_CMD) {
                s_state = SYS_STATE_SLEEP;
            }
            break;

        case SYS_STATE_SLEEP:
            if (ev == EV_WAKEUP) {
                s_state = SYS_STATE_WORK;
            }
            break;

        case SYS_STATE_ERROR:
            if (ev == EV_SCAN_START) {
                s_state = SYS_STATE_SCAN;
            }
            break;
    }
}
