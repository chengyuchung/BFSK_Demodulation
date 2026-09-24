/* app_frame_handler.c - 中继节点（透传） */
#include "app_frame_handler.h"

void app_frame_handler_init(void) {}

void app_frame_handler_on_frame(const frame_t *frame)
{
    /* 中继节点：直接透传，不解析内容 */
    /* TODO: 将帧重新发送（需要标记来源方向，避免回环） */
    (void)frame;
}
