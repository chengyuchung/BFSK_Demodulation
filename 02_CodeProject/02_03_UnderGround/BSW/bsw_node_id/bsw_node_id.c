/**
 * @file    bsw_node_id.c
 * @brief   节点设备地址模块 - BSW 层实现
 */

#include "bsw_node_id.h"

static uint8_t s_node_id = BSW_NODE_ID_INVALID;
static uint8_t s_upstream_addr = BSW_NODE_ADDR_UNKNOWN;    /* 上级节点地址 */
static uint8_t s_downstream_addr = BSW_NODE_ADDR_UNKNOWN;  /* 下级节点地址 */

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
        return BSW_NODE_ID_SLOT_INVALID;   /* 哨兵：未初始化，永不轮到 */
    }
    return (uint32_t)s_node_id * BSW_NODE_REPLY_STEP_MS;
}

void bsw_node_id_set_upstream(uint8_t addr)
{
    s_upstream_addr = addr;
}

uint8_t bsw_node_id_get_upstream(void)
{
    return s_upstream_addr;
}

void bsw_node_id_set_downstream(uint8_t addr)
{
    s_downstream_addr = addr;
}

uint8_t bsw_node_id_get_downstream(void)
{
    return s_downstream_addr;
}

bool bsw_node_id_can_accept_broadcast(void)
{
    /* 只要上级或下级地址有一个未知，就可以接收广播 */
    return (s_upstream_addr == BSW_NODE_ADDR_UNKNOWN) || 
           (s_downstream_addr == BSW_NODE_ADDR_UNKNOWN);
}
