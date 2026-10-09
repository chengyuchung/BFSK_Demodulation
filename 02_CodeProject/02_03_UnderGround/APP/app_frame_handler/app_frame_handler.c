/**
 * @file    app_frame_handler.c
 * @brief   帧业务处理层 - 实现
 *
 * 处理流程：
 *   app_node_fsm → app_frame_handler_dispatch() → 具体处理函数
 *
 * 当前已实现的处理函数：
 *   - _handle_query_frame()    QUERY 类型查询处理
 *     - PROTO_QUERY_VERSION    版本号查询（✅ 已完成，含发送）
 *     - PROTO_QUERY_TEMP_PRESS 温压查询（✅ 已完成，含发送）
 *     - PROTO_QUERY_BATTERY    电池查询（TODO）
 *     - PROTO_QUERY_FAULT      故障查询（TODO）
 *   - _handle_control_frame()  CONTROL 类型控制帧（TODO）
 *   - _handle_reply_frame()    REPLY 类型回复帧（TODO）
 *   - _handle_alarm_frame()    ALARM 类型警报帧（TODO）
 *
 * @dependency  app_frame_handler.h
 *              bsw_proto.h
 *              bsw_version.h
 *              bsw_node_id.h
 *              app_sensor.h
 *              bsw_bfsk_mod.h
 */

#include "app_frame_handler.h"

#include <string.h>
#include "bsw_proto.h"
#include "bsw_version.h"
#include "bsw_node_id.h"
#include "app_sensor.h"
#include "bsw_bfsk_mod.h"

/* ========== 内部函数声明 ========== */

static frame_handler_ret_t _handle_query_frame(const proto_frame_t *frame);
static frame_handler_ret_t _handle_control_frame(const proto_frame_t *frame);
static frame_handler_ret_t _handle_reply_frame(const proto_frame_t *frame);
static frame_handler_ret_t _handle_alarm_frame(const proto_frame_t *frame);

/* QUERY 子类型处理 */
static frame_handler_ret_t _handle_query_version(const proto_frame_t *frame);
static frame_handler_ret_t _handle_query_temp_press(const proto_frame_t *frame);
static frame_handler_ret_t _handle_query_battery(const proto_frame_t *frame);
static frame_handler_ret_t _handle_query_fault(const proto_frame_t *frame);

/* 辅助函数 */
static frame_handler_ret_t _send_nack(const proto_frame_t *frame, proto_nack_code_t err_code);

/* ========== 公共 API 实现 ========== */

void app_frame_handler_init(void)
{
    /* 当前无需要初始化的内容（各 BSW 模块自己负责 init）*/
}

frame_handler_ret_t app_frame_handler_dispatch(const proto_frame_t *frame)
{
    if (frame == NULL) {
        return FRAME_HANDLER_ERR_PARAM;
    }
    
    /* 地址过滤：DST 必须是本机地址 */
    uint8_t local_addr = bsw_node_id_get();
    if (frame->addr.dst != local_addr) {
        return FRAME_HANDLER_ERR_ADDR_MISMATCH;
    }
    
    /* 根据帧类型分发 */
    switch (frame->type_info.type) {
        case PROTO_TYPE_QUERY:
            return _handle_query_frame(frame);
            
        case PROTO_TYPE_CONTROL:
            return _handle_control_frame(frame);
            
        case PROTO_TYPE_REPLY:
            return _handle_reply_frame(frame);
            
        case PROTO_TYPE_ALARM:
            return _handle_alarm_frame(frame);
            
        default:
            return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
    }
}

/* ========== QUERY 类型帧处理 ========== */

static frame_handler_ret_t _handle_query_frame(const proto_frame_t *frame)
{
    /* 根据查询子类型分发 */
    switch (frame->type_info.msg_num) {
        case PROTO_QUERY_VERSION:
            return _handle_query_version(frame);
            
        case PROTO_QUERY_TEMP_PRESS:
            return _handle_query_temp_press(frame);
            
        case PROTO_QUERY_BATTERY:
            return _handle_query_battery(frame);
            
        case PROTO_QUERY_FAULT:
            return _handle_query_fault(frame);
            
        default:
            return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
    }
}

/**
 * @brief   处理版本号查询
 * @note    已实现：构造 REPLY_VERSION 帧并打包
 *          TODO: 实际发送（需要 BFSK 调制器）
 */
static frame_handler_ret_t _handle_query_version(const proto_frame_t *frame)
{
    const bsw_version_info_t *ver = bsw_version_get_info();
    
    /* 准备回复帧 */
    proto_frame_t reply;
    reply.addr.dst = frame->addr.src;  /* 回复给查询方 */
    reply.addr.src = bsw_node_id_get();
    reply.type_info.type = PROTO_TYPE_REPLY;
    reply.type_info.msg_num = PROTO_REPLY_VERSION;
    reply.seq = frame->seq;
    reply.payload_len = PROTO_VERSION_PAYLOAD_SIZE;
    
    /* 打包版本号（小端格式）*/
    reply.payload[0] = (uint8_t)((ver->sw_version_date >>  0) & 0xFFU);
    reply.payload[1] = (uint8_t)((ver->sw_version_date >>  8) & 0xFFU);
    reply.payload[2] = (uint8_t)((ver->sw_version_date >> 16) & 0xFFU);
    reply.payload[3] = (uint8_t)((ver->sw_version_date >> 24) & 0xFFU);
    reply.payload[4] = (uint8_t)((ver->hw_version_date >>  0) & 0xFFU);
    reply.payload[5] = (uint8_t)((ver->hw_version_date >>  8) & 0xFFU);
    reply.payload[6] = (uint8_t)((ver->hw_version_date >> 16) & 0xFFU);
    reply.payload[7] = (uint8_t)((ver->hw_version_date >> 24) & 0xFFU);
    
    /* 打包成线缆字节流 */
    uint8_t tx_buf[PROTO_FRAME_MAX];
    uint32_t tx_len = 0;
    proto_err_t err = proto_frame_pack(tx_buf, sizeof(tx_buf), &reply, &tx_len);
    
    if (err != PROTO_ERR_OK || tx_len == 0) {
        return FRAME_HANDLER_ERR_SEND_FAIL;
    }
    
    /* 调用 BFSK 调制器发送 */
    bsw_bfsk_mod_ret_t mod_ret = bsw_bfsk_mod_send(tx_buf, (uint16_t)tx_len);
    if (mod_ret != BSW_BFSK_MOD_OK) {
        return FRAME_HANDLER_ERR_SEND_FAIL;
    }
    
    return FRAME_HANDLER_OK;
}

/**
 * @brief   处理温压查询
 * @note    已实现：读取传感器数据，先发 ACK，再发温压数据帧
 *          TODO: 实际发送（需要 BFSK 调制器）
 */
static frame_handler_ret_t _handle_query_temp_press(const proto_frame_t *frame)
{
    /* 读取传感器数据 */
    app_sensor_data_t sensor_data;
    if (app_sensor_get_data(&sensor_data) != 0) {
        return FRAME_HANDLER_ERR_PARAM;
    }
    
    /* 检查数据有效性：至少温度或气压之一有效才回复 */
    if (!sensor_data.temp_valid && !sensor_data.press_valid) {
        /* 两个传感器都失败，发送 NACK */
        return _send_nack(frame, PROTO_NACK_BUSY);  /* TODO: 需要实现 _send_nack */
    }
    
    /* 准备回复帧 */
    proto_frame_t reply;
    reply.addr.dst = frame->addr.src;  /* 回复给查询方 */
    reply.addr.src = bsw_node_id_get();
    reply.type_info.type = PROTO_TYPE_REPLY;
    reply.type_info.msg_num = PROTO_REPLY_ACK;  /* 先发 ACK 确认收到 */
    reply.seq = frame->seq;
    reply.payload_len = 0;  /* ACK 无载荷 */
    
    /* 打包 ACK 帧 */
    uint8_t tx_buf[PROTO_FRAME_MAX];
    uint32_t tx_len = 0;
    proto_err_t err = proto_frame_pack(tx_buf, sizeof(tx_buf), &reply, &tx_len);
    
    if (err != PROTO_ERR_OK || tx_len == 0) {
        return FRAME_HANDLER_ERR_SEND_FAIL;
    }
    
    /* 发送 ACK 帧 */
    bsw_bfsk_mod_ret_t mod_ret = bsw_bfsk_mod_send(tx_buf, (uint16_t)tx_len);
    if (mod_ret != BSW_BFSK_MOD_OK) {
        return FRAME_HANDLER_ERR_SEND_FAIL;
    }
    
    /* TODO: 等待 ACK 发送完成后再发数据帧
     * 当前简化处理：假设 ACK 已发完（实际应该轮询 bsw_bfsk_mod_get_state()）
     */
    
    /* 准备温压数据帧（QUERY 类型，带数据载荷）*/
    reply.type_info.type = PROTO_TYPE_QUERY;
    reply.type_info.msg_num = PROTO_QUERY_TEMP_PRESS;
    reply.seq = (frame->seq + 1) & 0xFFU;  /* SEQ + 1 */
    reply.payload_len = PROTO_TEMP_PRESS_PAYLOAD_SIZE;
    
    /* 打包温压数据（小端格式）*/
    int16_t temp = sensor_data.temp_valid ? sensor_data.temperature_centi_c : 0;
    uint32_t press = sensor_data.press_valid ? sensor_data.pressure_pa : 0;
    
    reply.payload[0] = (uint8_t)((temp >>  0) & 0xFFU);
    reply.payload[1] = (uint8_t)((temp >>  8) & 0xFFU);
    reply.payload[2] = (uint8_t)((press >>  0) & 0xFFU);
    reply.payload[3] = (uint8_t)((press >>  8) & 0xFFU);
    reply.payload[4] = (uint8_t)((press >> 16) & 0xFFU);
    reply.payload[5] = (uint8_t)((press >> 24) & 0xFFU);
    
    /* 打包温压数据帧 */
    tx_len = 0;
    err = proto_frame_pack(tx_buf, sizeof(tx_buf), &reply, &tx_len);
    
    if (err != PROTO_ERR_OK || tx_len == 0) {
        return FRAME_HANDLER_ERR_SEND_FAIL;
    }
    
    /* 发送温压数据帧 */
    mod_ret = bsw_bfsk_mod_send(tx_buf, (uint16_t)tx_len);
    if (mod_ret != BSW_BFSK_MOD_OK) {
        return FRAME_HANDLER_ERR_SEND_FAIL;
    }
    
    return FRAME_HANDLER_OK;
}

/**
 * @brief   处理电池查询（TODO）
 */
static frame_handler_ret_t _handle_query_battery(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 读取电池电压并回复 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief   处理故障查询（TODO）
 */
static frame_handler_ret_t _handle_query_fault(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 读取故障状态并回复 */
    return FRAME_HANDLER_OK;
}

/* ========== CONTROL 类型帧处理（TODO）========== */

static frame_handler_ret_t _handle_control_frame(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 实现控制帧处理（设置休眠时长、阈值等）*/
    return FRAME_HANDLER_OK;
}

/* ========== REPLY 类型帧处理（TODO）========== */

static frame_handler_ret_t _handle_reply_frame(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 实现回复帧处理（ACK/NACK 等）*/
    return FRAME_HANDLER_OK;
}

/* ========== ALARM 类型帧处理（TODO）========== */

static frame_handler_ret_t _handle_alarm_frame(const proto_frame_t *frame)
{
    (void)frame;
    /* TODO: 实现警报帧处理 */
    return FRAME_HANDLER_OK;
}

/* ========== 辅助函数 ========== */

/**
 * @brief   发送 NACK 回复帧
 * @param   frame     原始请求帧
 * @param   err_code  错误码（见 proto_nack_code_t）
 * @retval  FRAME_HANDLER_OK / FRAME_HANDLER_ERR_SEND_FAIL
 */
static frame_handler_ret_t _send_nack(const proto_frame_t *frame, proto_nack_code_t err_code)
{
    proto_frame_t reply;
    reply.addr.dst = frame->addr.src;  /* 回复给发送方 */
    reply.addr.src = bsw_node_id_get();
    reply.type_info.type = PROTO_TYPE_REPLY;
    reply.type_info.msg_num = PROTO_REPLY_NACK;
    reply.seq = frame->seq;
    reply.payload_len = 1;
    reply.payload[0] = (uint8_t)err_code;
    
    /* 打包成线缆字节流 */
    uint8_t tx_buf[PROTO_FRAME_MAX];
    uint32_t tx_len = 0;
    proto_err_t err = proto_frame_pack(tx_buf, sizeof(tx_buf), &reply, &tx_len);
    
    if (err != PROTO_ERR_OK || tx_len == 0) {
        return FRAME_HANDLER_ERR_SEND_FAIL;
    }
    
    /* 调用 BFSK 调制器发送 */
    bsw_bfsk_mod_ret_t mod_ret = bsw_bfsk_mod_send(tx_buf, (uint16_t)tx_len);
    if (mod_ret != BSW_BFSK_MOD_OK) {
        return FRAME_HANDLER_ERR_SEND_FAIL;
    }
    
    return FRAME_HANDLER_OK;
}
