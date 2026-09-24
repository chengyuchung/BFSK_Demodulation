/* app_fsm.c - 中继节点 */
#include "app_fsm.h"
static sys_state_t s_state = SYS_STATE_INIT;
void app_fsm_init(void) { s_state = SYS_STATE_INIT; }
sys_state_t app_fsm_get_state(void) { return s_state; }
void app_fsm_dispatch(fsm_event_t ev)
{
    (void)ev;
    /* TODO: 中继节点状态机（透传模式，跳过 INIT 快速进入 WORK） */
}
