/**
 * @file    bsw_node_id.h
 * @brief   节点设备地址模块 - BSW 层
 *
 * 4 位地址固定分配表（协议 §4.3，井下次序 ground → underground）：
 *   0000         井上主机
 *   0001~1110    中继 1 ~ 中继 14
 *   1111         井下底节
 *
 * 应答槽时序（用户描述协议）：
 *   槽间距 = 200 ms    →  本节点槽起点 = my_id × 200
 *   槽宽度 = 500 ms    →  单次 1010 回发最长持续时间
 *
 * 本机地址：改下面 LOCAL_ADDRESS 一行即可。
 */

#ifndef BSW_NODE_ID_H
#define BSW_NODE_ID_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 地址分配表（ground → underground，固定 16 格） ========== */

#define BSW_NODE_ADDR_GROUND          0x0u   /* 井上主机     */
#define BSW_NODE_ADDR_RELAY_1         0x1u
#define BSW_NODE_ADDR_RELAY_2         0x2u
#define BSW_NODE_ADDR_RELAY_3         0x3u
#define BSW_NODE_ADDR_RELAY_4         0x4u
#define BSW_NODE_ADDR_RELAY_5         0x5u
#define BSW_NODE_ADDR_RELAY_6         0x6u
#define BSW_NODE_ADDR_RELAY_7         0x7u
#define BSW_NODE_ADDR_RELAY_8         0x8u
#define BSW_NODE_ADDR_RELAY_9         0x9u
#define BSW_NODE_ADDR_RELAY_10        0xAu
#define BSW_NODE_ADDR_RELAY_11        0xBu
#define BSW_NODE_ADDR_RELAY_12        0xCu
#define BSW_NODE_ADDR_RELAY_13        0xDu
#define BSW_NODE_ADDR_RELAY_14        0xEu
#define BSW_NODE_ADDR_UNDERGROUND     0xFu   /* 井下底节     */

#define BSW_NODE_ID_BITS         4u
#define BSW_NODE_ID_ADDR_MAX     ((1u << BSW_NODE_ID_BITS) - 1u)   /* 15 */
#define BSW_NODE_ID_INVALID      0xFFu

/* ========== 本机地址（单点配置：build 时改这一行） ========== */
#define LOCAL_ADDRESS    BSW_NODE_ADDR_UNDERGROUND

/* ========== 应答槽时序常量 ========== */

/** 槽间距：本节点槽起点 = my_id × BSW_NODE_REPLY_STEP_MS (单位 ms) */
#define BSW_NODE_REPLY_STEP_MS   200u

/** 单次 1010 回发最长持续时间 (单位 ms) */
#define BSW_NODE_REPLY_BURST_MS  500u

/* ========== API ========== */

void bsw_node_id_init(uint8_t hard_id);
uint8_t bsw_node_id_get(void);

/**
 * @brief   取本节点槽起点（相对扫频结束时刻的延迟，单位 ms）
 * @return  LOCAL_ADDRESS × BSW_NODE_REPLY_STEP_MS
 * @note    未 init 时返回 0xFFFFFFFF（哨兵：永不轮到）
 */
uint32_t bsw_node_id_get_reply_delay_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_NODE_ID_H */
