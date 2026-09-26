/**
 * @file    bsw_node_id.c
 * @brief   节点设备地址模块 - BSW 层实现
 */

#include "bsw_node_id.h"

static uint8_t s_node_id = BSW_NODE_ID_INVALID;

void bsw_node_id_init(uint8_t hard_id)
{
    if (hard_id > BSW_NODE_ID_ADDR_MAX) {
        s_node_id = BSW_NODE_ID_INVALID;
        return;
    }
    s_node_id = hard_id;
}

uint8_t bsw_node_id_get(void)
{
    return s_node_id;
}

uint32_t bsw_node_id_get_reply_delay_ms(void)
{
    if (s_node_id == BSW_NODE_ID_INVALID || s_node_id > BSW_NODE_ID_ADDR_MAX) {
        return 0xFFFFFFFFu;   /* 哨兵：未初始化，永不轮到 */
    }
    return (uint32_t)s_node_id * BSW_NODE_REPLY_STEP_MS;
}
