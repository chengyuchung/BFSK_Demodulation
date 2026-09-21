/**
 * @file    app_frame_handler.h
 * @brief   帧分发处理 - APP 层
 * @note    根据消息类型将帧分发给对应的业务处理函数
 *
 * @dependency  bsw_frame
 */

#ifndef APP_FRAME_HANDLER_H
#define APP_FRAME_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bsw_frame.h"

/* ========== 函数声明 ========== */

/**
 * @brief   初始化帧处理器
 */
void app_frame_handler_init(void);

/**
 * @brief   处理一帧数据
 * @param   frame   帧指针
 * @note    根据 msgType 分发到对应处理函数
 */
void app_frame_handler_on_frame(const frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* APP_FRAME_HANDLER_H */
