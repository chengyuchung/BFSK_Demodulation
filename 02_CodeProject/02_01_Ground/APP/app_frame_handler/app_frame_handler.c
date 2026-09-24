/* app_frame_handler.c - 井上节点 */
#include "app_frame_handler.h"
#include "app_fsm.h"
#include "mcal_uart.h"

void app_frame_handler_init(void)
{
    /* TODO: 注册各类型消息的处理回调 */
}

void app_frame_handler_on_frame(const frame_t *frame)
{
    switch (frame->msg_type) {
        case 0x01: /* 数据帧 */
            /* TODO: 解析传感器数据，更新显示 */
            break;

        case 0x02: /* 状态帧 */
            /* TODO: 更新节点状态 */
            break;

        case 0x03: /* 命令帧 */
            /* TODO: 执行控制命令 */
            app_fsm_dispatch(EV_FRAME_RECV);
            break;

        case 0x04: /* 应答帧 */
            /* TODO: 处理 ACK */
            break;

        case 0x07: /* 心跳帧 */
            /* TODO: 更新链路心跳计时 */
            break;

        default:
            mcal_uart_printf(UART_ID_DEBUG, "Unknown msg type: 0x%02X\r\n", frame->msg_type);
            break;
    }
}
