/* app_frame_handler.h - 井下节点 */
#ifndef APP_FRAME_HANDLER_H
#define APP_FRAME_HANDLER_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "bsw_frame.h"
void app_frame_handler_init(void);
void app_frame_handler_on_frame(const frame_t *frame);
#ifdef __cplusplus
}
#endif
#endif /* APP_FRAME_HANDLER_H */
