/**
 * @file    app_frame_handler.c
 * @brief   帧业务处理层 - 实现
 *
 * 处理流程（单一流程）：
 *   接收帧 → 判断类型 → 执行业务逻辑 → 组装回复帧 → 调制发送 → 返回结果
 *
 * 当前已实现的处理函数：
 *   - _handle_query_frame()    QUERY 类型查询处理
 *     - PROTO_QUERY_VERSION    版本号查询（✅ 已完成）
 *     - PROTO_QUERY_TEMP_PRESS 温压查询（✅ 已完成）
 *     - PROTO_QUERY_BATTERY    电池查询（TODO）
 *     - PROTO_QUERY_FAULT      故障查询（TODO）
 *   - _handle_control_frame()  CONTROL 类型控制帧（TODO）
 *   - _handle_ask_frame()      ASK 类型询问/应答帧（TODO）
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

/* ========== 配置宏定义 ========== */

/** 温压数据有效性检查 - 至少一个传感器有效 */
#define APP_SENSOR_MIN_VALID_COUNT          (1U)

/** 单次调度最多组装的回复帧数（ACK + 数据帧） */
#define APP_FRAME_HANDLER_MAX_REPLY_FRAMES  (2U)

/* ========== 内部类型定义 ========== */

/**
 * @brief QUERY 子类型处理函数原型
 * @param frame 接收到的查询帧
 * @param p_reply_buf 回复帧缓冲区（可能需要1-2个帧）
 * @param p_reply_count 输出：实际需要回复的帧数量
 * @return 处理结果
 */
typedef frame_handler_ret_t (*query_handler_t)(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);

/**
 * @brief QUERY 处理表项
 */
typedef struct
{
    uint8_t msg_num;                /**< 子类型编号（proto_query_msg_t）*/
    query_handler_t handler;        /**< 处理函数 */
    const char *p_name;             /**< 名称（调试用）*/
} QueryHandlerEntry_t;

/* ========== 内部函数声明 ========== */

static frame_handler_ret_t _handle_query_frame(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);
static frame_handler_ret_t _handle_control_frame(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count);
static frame_handler_ret_t _handle_ask_frame(const proto_frame_t *frame,
                                             proto_frame_t *p_reply_buf,
                                             uint8_t *p_reply_count);
static frame_handler_ret_t _handle_alarm_frame(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);

/* QUERY 子类型处理 */
static frame_handler_ret_t _handle_query_version(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count);
static frame_handler_ret_t _handle_query_temp_press(const proto_frame_t *frame,
                                                    proto_frame_t *p_reply_buf,
                                                    uint8_t *p_reply_count);
static frame_handler_ret_t _handle_query_battery(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count);
static frame_handler_ret_t _handle_query_fault(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);

/* 通用辅助函数 */
static frame_handler_ret_t _send_frames(const proto_frame_t *p_frames, uint8_t count);
static void _make_ack_frame(const proto_frame_t *req_frame, proto_frame_t *p_ack_frame);
static void _make_nack_frame(const proto_frame_t *req_frame,
                             proto_nack_code_t err_code,
                             proto_frame_t *p_nack_frame);

/* ========== QUERY 处理函数表 ========== */

/**
 * @brief QUERY 子类型处理表
 * @note 新增 QUERY 子类型时，在此表添加一项即可，无需修改 _handle_query_frame()
 */
static const QueryHandlerEntry_t s_query_handler_table[] =
{
    /* msg_num,                handler,                        name */
    { PROTO_QUERY_VERSION,     _handle_query_version,          "VERSION" },
    { PROTO_QUERY_TEMP_PRESS,  _handle_query_temp_press,       "TEMP_PRESS" },
    { PROTO_QUERY_BATTERY,     _handle_query_battery,          "BATTERY" },
    { PROTO_QUERY_FAULT,       _handle_query_fault,            "FAULT" },
};

#define QUERY_HANDLER_TABLE_SIZE  (sizeof(s_query_handler_table) / sizeof(s_query_handler_table[0]))

/* ========== 公共 API 实现 ========== */

void app_frame_handler_init(void)
{
    /* 当前无需要初始化的内容（各 BSW 模块自己负责 init）*/
}

frame_handler_ret_t app_frame_handler_dispatch(const proto_frame_t *frame)
{
    if (frame == NULL)
    {
        return FRAME_HANDLER_ERR_PARAM;
    }
    
    /* 地址过滤：
     * 1. 目的地址 == 本机地址（单播）
     * 2. 目的地址 == 广播地址（0xF）且（上级或下级地址未知）
     * 
     * 为什么：握手阶段需要接收广播帧，链路建立后自动拒绝广播
     */
    uint8_t local_addr = bsw_node_id_get();
    bool accept = (frame->addr.dst == local_addr);
    
    if (!accept && (frame->addr.dst == BSW_NODE_ADDR_BROADCAST))
    {
        accept = bsw_node_id_can_accept_broadcast();
    }
    
    if (!accept)
    {
        return FRAME_HANDLER_ERR_ADDR_MISMATCH;
    }
    
    /* 准备回复帧缓冲区 */
    proto_frame_t reply_buf[APP_FRAME_HANDLER_MAX_REPLY_FRAMES];
    uint8_t reply_count = 0;
    frame_handler_ret_t ret;
    
    /* 根据帧类型分发 */
    switch (frame->type_info.type)
    {
        case PROTO_TYPE_QUERY:
            ret = _handle_query_frame(frame, reply_buf, &reply_count);
            break;
            
        case PROTO_TYPE_CONTROL:
            ret = _handle_control_frame(frame, reply_buf, &reply_count);
            break;
            
        case PROTO_TYPE_ASK:
            ret = _handle_ask_frame(frame, reply_buf, &reply_count);
            break;
            
        case PROTO_TYPE_ALARM:
            ret = _handle_alarm_frame(frame, reply_buf, &reply_count);
            break;
            
        default:
            return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
    }
    
    /* 如果业务处理成功且有需要发送的回复帧，则发送 */
    if (ret == FRAME_HANDLER_OK && reply_count > 0)
    {
        ret = _send_frames(reply_buf, reply_count);
    }
    
    return ret;
}

/* ========== QUERY 类型帧处理 ========== */

/**
 * @brief 处理 QUERY 类型帧 - 使用函数表模式
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 为什么用函数表：
 * 1. 添加新 QUERY 子类型时只需在表中添加，无需修改此函数
 * 2. 每个子类型处理逻辑独立，便于单元测试
 * 3. 表项集中管理，便于查看所有支持的查询类型
 */
static frame_handler_ret_t _handle_query_frame(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count)
{
    /* 在处理表中查找对应的处理函数 */
    for (uint16_t i = 0; i < QUERY_HANDLER_TABLE_SIZE; i++)
    {
        if (s_query_handler_table[i].msg_num == frame->type_info.msg_num)
        {
            return s_query_handler_table[i].handler(frame, p_reply_buf, p_reply_count);
        }
    }
    
    /* 未找到对应的处理函数 */
    return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
}

/**
 * @brief 处理版本号查询
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 读取本机版本信息
 * 2. 构造 QUERY 类型回复帧（携带版本数据）
 * 3. 设置输出参数
 * 4. 返回（由上层统一发送）
 */
static frame_handler_ret_t _handle_query_version(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count)
{
    const bsw_version_info_t *p_ver = bsw_version_get_info();
    
    /* 准备回复帧：QUERY 类型，携带版本载荷 */
    proto_frame_t *p_reply = &p_reply_buf[0];
    p_reply->addr.dst = frame->addr.src;  /* 回复给查询方 */
    p_reply->addr.src = bsw_node_id_get();
    p_reply->type_info.type = PROTO_TYPE_QUERY;
    p_reply->type_info.msg_num = PROTO_QUERY_VERSION;
    p_reply->seq = frame->seq;
    p_reply->payload_len = PROTO_VERSION_PAYLOAD_SIZE;
    
    /* 打包版本号（小端格式）*/
    p_reply->payload[0] = (uint8_t)((p_ver->sw_version_date >>  0) & 0xFFU);
    p_reply->payload[1] = (uint8_t)((p_ver->sw_version_date >>  8) & 0xFFU);
    p_reply->payload[2] = (uint8_t)((p_ver->sw_version_date >> 16) & 0xFFU);
    p_reply->payload[3] = (uint8_t)((p_ver->sw_version_date >> 24) & 0xFFU);
    p_reply->payload[4] = (uint8_t)((p_ver->hw_version_date >>  0) & 0xFFU);
    p_reply->payload[5] = (uint8_t)((p_ver->hw_version_date >>  8) & 0xFFU);
    p_reply->payload[6] = (uint8_t)((p_ver->hw_version_date >> 16) & 0xFFU);
    p_reply->payload[7] = (uint8_t)((p_ver->hw_version_date >> 24) & 0xFFU);
    
    *p_reply_count = 1;  /* 只需回复1个帧 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理温压查询
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 读取传感器数据
 * 2. 检查有效性（至少一个传感器有效）
 * 3. 先组装 ACK 确认帧
 * 4. 再组装温压数据帧
 * 5. 返回2个帧（由上层统一发送）
 */
static frame_handler_ret_t _handle_query_temp_press(const proto_frame_t *frame,
                                                    proto_frame_t *p_reply_buf,
                                                    uint8_t *p_reply_count)
{
    /* 读取传感器数据 */
    app_sensor_data_t sensor_data;
    if (app_sensor_get_data(&sensor_data) != 0)
    {
        return FRAME_HANDLER_ERR_PARAM;
    }
    
    /* 检查数据有效性：至少温度或气压之一有效才回复 */
    if (!sensor_data.temp_valid && !sensor_data.press_valid)
    {
        /* 两个传感器都失败，组装 NACK */
        _make_nack_frame(frame, PROTO_NACK_SENSOR_FAIL, &p_reply_buf[0]);
        *p_reply_count = 1;
        return FRAME_HANDLER_OK;
    }
    
    /* 第1个帧：ACK 确认收到 */
    _make_ack_frame(frame, &p_reply_buf[0]);
    
    /* 第2个帧：温压数据帧（QUERY 类型，带数据载荷）*/
    proto_frame_t *p_data_frame = &p_reply_buf[1];
    p_data_frame->addr.dst = frame->addr.src;  /* 回复给查询方 */
    p_data_frame->addr.src = bsw_node_id_get();
    p_data_frame->type_info.type = PROTO_TYPE_QUERY;
    p_data_frame->type_info.msg_num = PROTO_QUERY_TEMP_PRESS;
    p_data_frame->seq = (frame->seq + 1) & 0xFFU;  /* SEQ + 1 */
    p_data_frame->payload_len = PROTO_TEMP_PRESS_PAYLOAD_SIZE;
    
    /* 打包温压数据（小端格式）*/
    int16_t temp = sensor_data.temp_valid ? sensor_data.temperature_centi_c : 0;
    uint32_t press = sensor_data.press_valid ? sensor_data.pressure_pa : 0;
    
    p_data_frame->payload[0] = (uint8_t)((temp >>  0) & 0xFFU);
    p_data_frame->payload[1] = (uint8_t)((temp >>  8) & 0xFFU);
    p_data_frame->payload[2] = (uint8_t)((press >>  0) & 0xFFU);
    p_data_frame->payload[3] = (uint8_t)((press >>  8) & 0xFFU);
    p_data_frame->payload[4] = (uint8_t)((press >> 16) & 0xFFU);
    p_data_frame->payload[5] = (uint8_t)((press >> 24) & 0xFFU);
    
    *p_reply_count = 2;  /* 回复 ACK + 数据帧 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理电池查询（TODO）
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 */
static frame_handler_ret_t _handle_query_battery(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 读取电池电压并组装回复 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理故障查询（TODO）
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 */
static frame_handler_ret_t _handle_query_fault(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 读取故障状态并组装回复 */
    return FRAME_HANDLER_OK;
}

/* ========== CONTROL 类型帧处理（TODO）========== */

/**
 * @brief 处理 CONTROL 类型帧（TODO）
 * @param frame 控制帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 */
static frame_handler_ret_t _handle_control_frame(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现控制帧处理（设置休眠时长、阈值等）*/
    return FRAME_HANDLER_OK;
}

/* ========== ASK 类型帧处理（询问/应答）========== */

/**
 * @brief 处理 ASK 类型帧（询问/应答）
 * @param frame 询问/应答帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * ASK 帧处理：
 * - MSG_NUM < 0x8：询问帧（例如 IDLE_CONFIRM）
 * - MSG_NUM >= 0x8：应答帧（例如 ACK、NACK）
 */
static frame_handler_ret_t _handle_ask_frame(const proto_frame_t *frame,
                                             proto_frame_t *p_reply_buf,
                                             uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 ASK 类型帧处理（握手、ACK/NACK 等）*/
    return FRAME_HANDLER_OK;
}

/* ========== ALARM 类型帧处理（TODO）========== */

/**
 * @brief 处理 ALARM 类型帧（TODO）
 * @param frame 警报帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 */
static frame_handler_ret_t _handle_alarm_frame(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现警报帧处理 */
    return FRAME_HANDLER_OK;
}

/* ========== 通用辅助函数 ========== */

/**
 * @brief 统一发送一组帧
 * @param p_frames 帧数组
 * @param count 帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 逐个帧进行打包
 * 2. 调用 BFSK 调制器发送
 * 3. 任何帧发送失败则返回错误
 */
static frame_handler_ret_t _send_frames(const proto_frame_t *p_frames, uint8_t count)
{
    if (p_frames == NULL || count == 0 || count > APP_FRAME_HANDLER_MAX_REPLY_FRAMES)
    {
        return FRAME_HANDLER_ERR_PARAM;
    }
    
    for (uint8_t i = 0; i < count; i++)
    {
        /* 打包帧为线缆字节流 */
        uint8_t tx_buf[PROTO_FRAME_MAX];
        uint32_t tx_len = 0;
        proto_err_t err = proto_frame_pack(tx_buf, sizeof(tx_buf), &p_frames[i], &tx_len);
        
        if (err != PROTO_ERR_OK || tx_len == 0)
        {
            return FRAME_HANDLER_ERR_SEND_FAIL;
        }
        
        /* 调用 BFSK 调制器发送 */
        bsw_bfsk_mod_ret_t mod_ret = bsw_bfsk_mod_send(tx_buf, (uint16_t)tx_len);
        if (mod_ret != BSW_BFSK_MOD_OK)
        {
            return FRAME_HANDLER_ERR_SEND_FAIL;
        }
    }
    
    return FRAME_HANDLER_OK;
}

/**
 * @brief 组装 ACK 回复帧（通用）
 * @param req_frame 原始请求帧
 * @param p_ack_frame 输出：ACK 帧
 * 
 * @details
 * ACK 帧格式：
 * - DST = 接收帧的 SRC（回复给发送方）
 * - SRC = 本机地址
 * - TYPE = PROTO_TYPE_ASK
 * - MSG_NUM = PROTO_ASK_ACK
 * - SEQ = 接收帧的 SEQ
 * - PAYLOAD = 无
 * 
 * @note 为什么目的地址是 req_frame->addr.src：
 *       ACK 回复必须发给原始发送方，无论是上级、下级还是广播帧
 */
static void _make_ack_frame(const proto_frame_t *req_frame, proto_frame_t *p_ack_frame)
{
    p_ack_frame->addr.dst = req_frame->addr.src;  /* 回复给原始发送方 */
    p_ack_frame->addr.src = bsw_node_id_get();
    p_ack_frame->type_info.type = PROTO_TYPE_ASK;
    p_ack_frame->type_info.msg_num = PROTO_ASK_ACK;
    p_ack_frame->seq = req_frame->seq;
    p_ack_frame->payload_len = 0;  /* ACK 无载荷 */
}

/**
 * @brief 组装 NACK 回复帧
 * @param req_frame 原始请求帧
 * @param err_code 错误码（见 proto_nack_code_t）
 * @param p_nack_frame 输出：NACK 帧
 * 
 * @details
 * NACK 帧格式：
 * - TYPE = PROTO_TYPE_ASK
 * - MSG_NUM = PROTO_ASK_NACK
 * - PAYLOAD[0] = err_code
 */
static void _make_nack_frame(const proto_frame_t *req_frame,
                             proto_nack_code_t err_code,
                             proto_frame_t *p_nack_frame)
{
    p_nack_frame->addr.dst = req_frame->addr.src;  /* 回复给发送方 */
    p_nack_frame->addr.src = bsw_node_id_get();
    p_nack_frame->type_info.type = PROTO_TYPE_ASK;
    p_nack_frame->type_info.msg_num = PROTO_ASK_NACK;
    p_nack_frame->seq = req_frame->seq;
    p_nack_frame->payload_len = 1;
    p_nack_frame->payload[0] = (uint8_t)err_code;
}
