/**
 * @file    app_frame_handler.h
 * @brief   帧业务处理层 - 独立处理各类协议帧
 *
 * 职责：
 *   - 接收协议层解析好的帧对象（proto_frame_t）
 *   - 根据帧类型分发到对应的处理函数
 *   - 处理具体业务逻辑（版本查询、温压查询、控制命令等）
 *   - 构造回复帧并发送
 *
 * 分层关系：
 *   app_node_fsm（状态机）
 *       ↓ 调用
 *   app_frame_handler（帧业务处理）← 本模块
 *       ↓ 使用
 *   bsw_proto（协议帧打包/解包）
 *       ↓ 使用
 *   bsw_bfsk_demod（物理层 bit 流）
 *
 * @dependency  bsw_proto.h        (帧结构定义)
 *              bsw_version.h      (版本信息)
 *              bsw_node_id.h      (节点地址)
 *
 * @usage
 *   // 初始化
 *   app_frame_handler_init();
 *   
 *   // 在 app_node_fsm 中接收到帧后调用
 *   proto_frame_t frame;
 *   proto_rx_take_frame(&frame);
 *   app_frame_handler_dispatch(&frame);
 */

#ifndef APP_FRAME_HANDLER_H
#define APP_FRAME_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsw_proto.h"

/* ========== 返回码 ========== */

typedef enum {
    FRAME_HANDLER_OK                = 0,   /* 处理成功 */
    FRAME_HANDLER_ERR_ADDR_MISMATCH = -1,  /* 地址不匹配 */
    FRAME_HANDLER_ERR_UNKNOWN_TYPE  = -2,  /* 未知帧类型 */
    FRAME_HANDLER_ERR_SEND_FAIL     = -3,  /* 发送失败 */
    FRAME_HANDLER_ERR_PARAM         = -4,  /* 参数错误 */
} frame_handler_ret_t;

/* ========== 公共 API ========== */

/**
 * @brief   初始化帧处理器
 * @note    在系统启动时调用
 */
void app_frame_handler_init(void);

/**
 * @brief   分发帧到对应处理函数
 * @param   frame  待处理的帧对象
 * @return  处理结果
 * @note    根据 frame->type_info.type 分发到具体处理函数
 */
frame_handler_ret_t app_frame_handler_dispatch(const proto_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* APP_FRAME_HANDLER_H */
