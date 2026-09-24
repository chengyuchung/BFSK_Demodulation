/**
 * @file    app_main.h - 井下节点
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
#define NODE_ADDR_UNDERGROUND  0x03
typedef enum {
    SYS_STATE_INIT, SYS_STATE_SCAN, SYS_STATE_LINKED,
    SYS_STATE_WORK, SYS_STATE_SLEEP, SYS_STATE_ERROR
} sys_state_t;
void app_main_init_init(void);
#ifdef __cplusplus
}
#endif
#endif /* APP_MAIN_H */
