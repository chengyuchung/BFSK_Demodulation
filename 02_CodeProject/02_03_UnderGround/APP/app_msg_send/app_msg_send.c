/**
 * @file    app_msg_send.c
 * @brief   消息发送模块实现 - 管理发送缓冲、重传和 ACK 处理
 *
 * 实现说明：
 * - 使用单个缓冲区存储待发送帧
 * - 记录发送时间戳、重试次数、超时时间
 * - 周期检测超时，自动重传
 * - 收到 ACK/NACK 时更新状态
 *
 * 状态机：
 *   空闲（is_waiting_ack=false）
 *       ↓ app_msg_send_frame()
 *   等待 ACK（is_waiting_ack=true, retry_count=0）
 *       ↓ 超时未收到
 *   重传（retry_count++, 重新发送）
 *       ↓ 再次等待 ACK
 *   等待 ACK（is_waiting_ack=true, retry_count>0）
 *       ↓ 收到 ACK 或达到最大重试
 *   空闲（清空缓冲区）
 */

#include "app_msg_send.h"
#include "bsw_proto.h"
#include "bsw_bfsk_mod.h"
#include <string.h>

/* ========== 私有类型定义 ========== */

/**
 * @brief 发送缓冲区状态结构体
 * @rationale: 集中管理发送状态和打包的字节流，便于维护和扩展
 */
typedef struct
{
    uint8_t packed_buf[PROTO_FRAME_MAX];    /* 打包后的字节流（重传时直接用） */
    uint16_t packed_len;                    /* 打包后的字节数 */
    uint8_t retry_count;                    /* 重传次数（0 表示首次） */
    uint32_t last_send_time_ms;             /* 上一次发送的时间戳 */
    uint32_t timeout_ms;                    /* 超时时间 */
    uint8_t seq;                            /* 帧序列号（用于 ACK 匹配） */
    bool is_waiting_ack;                    /* 是否在等待 ACK */
} TxPendingBuffer_t;

/* ========== 模块级静态变量 ========== */

/**
 * @brief 发送缓冲区
 * @rationale: 需要在函数调用间持久化，必须是静态
 * @thread-safe: 仅从主任务访问，无中断或多任务竞争
 */
static TxPendingBuffer_t s_tx_buffer = {0};

/**
 * @brief 模块初始化标志
 * @rationale: 防止未初始化时使用模块
 */
static bool s_is_initialized = false;

/* ========== 私有函数 ========== */

/**
 * @brief 清空发送缓冲区
 * @return void
 *
 * @details
 * 为什么：将缓冲区恢复到初始空闲状态
 * 何时调用：
 * - 初始化时
 * - 收到 ACK 后
 * - 重传超时后
 */
static void msg_send_clear_buffer(void)
{
    s_tx_buffer.is_waiting_ack = false;
    s_tx_buffer.retry_count = 0;
    s_tx_buffer.last_send_time_ms = 0;
    s_tx_buffer.timeout_ms = 0;
    s_tx_buffer.packed_len = 0;
    s_tx_buffer.seq = 0;
    
    /* 为什么：清除缓冲区数据，防止误用旧数据 */
    memset(s_tx_buffer.packed_buf, 0, sizeof(s_tx_buffer.packed_buf));
}

/**
 * @brief 将协议帧打包并发送到调制器
 * @param p_frame 待发送的帧
 * @return msg_send_ret_t - MSG_SEND_OK 表示发送成功
 *
 * @details
 * 为什么单独成函数：
 * - 首次发送和重传都需要调用
 * - 便于集中处理打包和调制发送的错误
 * - 对应架构分层：APP → 打包 → 调制 → 物理
 *
 * 分层职责：
 * 1. 本函数：frame → 打包 → 字节流
 * 2. bsw_bfsk_mod_send：字节流 → 频率切换 → 物理信号
 */
static msg_send_ret_t msg_send_pack_and_send(const proto_frame_t *p_frame)
{
    if (p_frame == NULL)
    {
        return MSG_SEND_ERR_PARAM;
    }
    
    /* 为什么：将 proto_frame_t 打包成线缆字节流 */
    proto_err_t pack_ret = proto_frame_pack(
        s_tx_buffer.packed_buf,
        sizeof(s_tx_buffer.packed_buf),
        p_frame,
        (uint32_t *)&s_tx_buffer.packed_len
    );
    
    if (pack_ret != PROTO_ERR_OK)
    {
        return MSG_SEND_ERR_PARAM;  /* 打包失败，参数或结构体有问题 */
    }
    
    /* 为什么：调用调制器的发送函数，注意参数类型的匹配 */
    bsw_bfsk_mod_ret_t mod_ret = bsw_bfsk_mod_send(
        s_tx_buffer.packed_buf,
        s_tx_buffer.packed_len
    );
    
    if (mod_ret != BSW_BFSK_MOD_OK)
    {
        return MSG_SEND_ERR_MOD_FAIL;  /* 调制器拒绝发送 */
    }
    
    return MSG_SEND_OK;
}

/* ========== 公共 API 实现 ========== */

void app_msg_send_init(void)
{
    /* 为什么：初始化缓冲区和标志 */
    msg_send_clear_buffer();
    s_is_initialized = true;
}

msg_send_ret_t app_msg_send_frame(const proto_frame_t *p_frame)
{
    /* 参数检查 */
    if (!s_is_initialized)
    {
        return MSG_SEND_ERR_NOT_INIT;
    }
    
    if (p_frame == NULL)
    {
        return MSG_SEND_ERR_PARAM;
    }
    
    /* 为什么：检查缓冲区是否被占用，防止数据丢失 */
    if (s_tx_buffer.is_waiting_ack)
    {
        return MSG_SEND_ERR_BUSY;
    }
    
    /* 为什么：调制器发送必须先成功，才能保存到缓冲区等待 ACK */
    msg_send_ret_t result = msg_send_pack_and_send(p_frame);
    if (result != MSG_SEND_OK)
    {
        return result;
    }
    
    /* 为什么：保存序列号，内部使用固定的超时时间 MSG_SEND_DEFAULT_TIMEOUT_MS */
    s_tx_buffer.seq = p_frame->seq;
    s_tx_buffer.retry_count = 0;
    s_tx_buffer.last_send_time_ms = 0;  /* 稍后由 tick 函数设置 */
    s_tx_buffer.timeout_ms = MSG_SEND_DEFAULT_TIMEOUT_MS;  /* 固定为 5 秒 */
    s_tx_buffer.is_waiting_ack = true;
    
    return MSG_SEND_OK;
}

msg_send_ret_t app_msg_send_ack(const proto_frame_t *p_frame)
{
    /* 参数检查 */
    if (!s_is_initialized)
    {
        return MSG_SEND_ERR_NOT_INIT;
    }
    
    if (p_frame == NULL)
    {
        return MSG_SEND_ERR_PARAM;
    }
    
    /* 为什么：ACK/NACK 帧不需要等待和重传，直接打包发送 */
    uint8_t tx_buf[PROTO_FRAME_MAX];
    uint32_t tx_len = 0;
    
    proto_err_t pack_ret = proto_frame_pack(
        tx_buf,
        sizeof(tx_buf),
        p_frame,
        &tx_len
    );
    
    if (pack_ret != PROTO_ERR_OK)
    {
        return MSG_SEND_ERR_PARAM;
    }
    
    /* 为什么：调用调制器发送，发完即走，不保存到缓冲区 */
    bsw_bfsk_mod_ret_t mod_ret = bsw_bfsk_mod_send(tx_buf, (uint16_t)tx_len);
    
    if (mod_ret != BSW_BFSK_MOD_OK)
    {
        return MSG_SEND_ERR_MOD_FAIL;
    }
    
    return MSG_SEND_OK;
}

void app_msg_send_notify_ack(uint8_t seq)
{
    if (!s_is_initialized)
    {
        return;
    }
    
    /* 为什么：只有在等待 ACK 时才处理 */
    if (!s_tx_buffer.is_waiting_ack)
    {
        return;
    }
    
    /* 为什么：验证 ACK 是否针对当前缓冲区的帧 */
    if (seq == s_tx_buffer.seq)
    {
        /* 匹配 - 清空缓冲区，表示成功 */
        msg_send_clear_buffer();
    }
    /* 否则：不匹配，可能是迟到的 ACK，忽略 */
}

void app_msg_send_notify_nack(uint8_t seq)
{
    if (!s_is_initialized)
    {
        return;
    }
    
    /* 为什么：只有在等待 ACK 时才处理 */
    if (!s_tx_buffer.is_waiting_ack)
    {
        return;
    }
    
    /* 为什么：验证 NACK 是否针对当前缓冲区的帧 */
    if (seq != s_tx_buffer.seq)
    {
        return;  /* 不匹配，忽略 */
    }
    
    /* 匹配 - 触发重传（如未达到最大次数） */
    if (s_tx_buffer.retry_count < MSG_SEND_MAX_RETRY)
    {
        /* 为什么：立即重传打包后的字节流 */
        bsw_bfsk_mod_ret_t mod_ret = bsw_bfsk_mod_send(
            s_tx_buffer.packed_buf,
            s_tx_buffer.packed_len
        );
        
        if (mod_ret == BSW_BFSK_MOD_OK)
        {
            s_tx_buffer.retry_count++;
            s_tx_buffer.last_send_time_ms = 0;  /* 标记为刚发送 */
        }
        /* 否则：调制器故障，等待下一个 tick 中的超时处理 */
    }
    else
    {
        /* 为什么：达到最大重试次数，放弃并清空缓冲区 */
        msg_send_clear_buffer();
        
        /* TODO: 可在此处添加警报或日志，通知应用发送失败 */
    }
}

void app_msg_send_tick(uint32_t now_tick)
{
    if (!s_is_initialized)
    {
        return;
    }
    
    /* 为什么：仅在等待 ACK 时进行超时检测 */
    if (!s_tx_buffer.is_waiting_ack)
    {
        return;
    }
    
    /* 为什么：首次发送时记录时间戳 */
    if (s_tx_buffer.last_send_time_ms == 0)
    {
        s_tx_buffer.last_send_time_ms = now_tick;
        return;
    }
    
    /* 为什么：计算已等待时间，检测是否超时 */
    uint32_t elapsed_ms = now_tick - s_tx_buffer.last_send_time_ms;
    
    if (elapsed_ms >= s_tx_buffer.timeout_ms)
    {
        /* 超时 */
        if (s_tx_buffer.retry_count < MSG_SEND_MAX_RETRY)
        {
            /* 为什么：还可以重传，直接重发打包好的字节流 */
            bsw_bfsk_mod_ret_t mod_ret = bsw_bfsk_mod_send(
                s_tx_buffer.packed_buf,
                s_tx_buffer.packed_len
            );
            
            if (mod_ret == BSW_BFSK_MOD_OK)
            {
                s_tx_buffer.retry_count++;
                s_tx_buffer.last_send_time_ms = now_tick;  /* 更新时间戳 */
                
                /* 可选：日志记录重传信息 */
                /* LOG_INFO("MSG_SEND: Retry %d for seq=%d", s_tx_buffer.retry_count, s_tx_buffer.frame.seq); */
            }
        }
        else
        {
            /* 为什么：达到最大重试次数，放弃 */
            msg_send_clear_buffer();
            
            /* 可选：日志记录失败信息 */
            /* LOG_ERROR("MSG_SEND: Timeout after %d retries", MSG_SEND_MAX_RETRY); */
        }
    }
}

bool app_msg_send_is_busy(void)
{
    if (!s_is_initialized)
    {
        return false;
    }
    
    return s_tx_buffer.is_waiting_ack;
}

msg_send_ret_t app_msg_send_get_buffered_frame(proto_frame_t *p_frame)
{
    if (!s_is_initialized)
    {
        return MSG_SEND_ERR_NOT_INIT;
    }
    
    if (p_frame == NULL)
    {
        return MSG_SEND_ERR_PARAM;
    }
    
    /* 为什么：已不再保存原始帧，仅保存打包后的字节流 */
    /* 此接口已废弃，无法提供原始帧数据 */
    (void)p_frame;  /* 避免编译警告 */
    
    return MSG_SEND_ERR_PARAM;  /* 不支持此操作 */
}
