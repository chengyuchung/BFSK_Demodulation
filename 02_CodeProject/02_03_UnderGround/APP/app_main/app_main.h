/**
 * @file    app_main.h - 井下节点 APP 层入口
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 兼容别名（原有模块依赖 sys_state_t） ========== */
/* 旧 sys_state_t 已迁移至 app_node_fsm.h 中的 node_state_t，
 * 为避免 app_sensor / app_task 大量改 include，此处保留别名映射。
 * 新代码请直接 include "app_node_fsm.h" 并使用 node_state_t。
 *
 * 注：使用相对路径 include，因为 Keil 项目里 APP 各子模块
 * 尚未加入 Include Path。后续若统一加 include path 可改回短名。 */
#include "../app_node_fsm/app_node_fsm.h"   /* node_state_t 在此定义 */

#define SYS_STATE_INIT    NODE_BOOT
#define SYS_STATE_SCAN    NODE_SCAN_LISTEN
#define SYS_STATE_LINKED  NODE_LINKED
#define SYS_STATE_WORK    NODE_LINKED  /* 工作模式对应 LINKED（频对已锁定） */
#define SYS_STATE_SLEEP   NODE_SLEEP
#define SYS_STATE_ERROR   NODE_FAULT

/* ========== APP 层入口原型 ========== */
/* 实现见 app_main.c。
 * 调用方（main.c、测试桩）只需 include 本头即可拿到声明。 */
void app_main_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
