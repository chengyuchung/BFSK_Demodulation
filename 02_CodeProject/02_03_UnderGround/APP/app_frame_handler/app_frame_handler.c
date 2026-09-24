/* app_frame_handler.c - 井下节点 */
#include "app_frame_handler.h"
#include "app_fsm.h"

void app_frame_handler_init(void) {}

void app_frame_handler_on_frame(const frame_t *frame)
{
    switch (frame->msg_type) {
        case 0x01: /* 数据帧 - 井下节点不上报数据，仅记录 */
            break;

        case 0x03: /* 命令帧 - 执行休眠等 */
            if (frame->payload_len > 0 && frame->payload[0] == 'S') {
                app_fsm_dispatch(EV_SLEEP_CMD);
            }
            break;

        case 0x07: /* 心跳 - 保持链路 */
            app_fsm_dispatch(EV_LINK_UP);
            break;

        default:
            break;
    }
}
