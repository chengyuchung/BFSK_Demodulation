/**
 * @file    app_msg_freq.c
 * @brief   消息频率管理模块实现
 * 
 * @details
 * 负责管理 19 个通信频点的静态数据，并提供频率验证和转换工具。
 */

#include "app_msg_freq.h"
#include <string.h>

/* ============================================================================ */
/* 静态数据 - 19 个频点表                                                        */
/* ============================================================================ */

/**
 * @brief 通信频点静态表
 * @details 19 个频点：125, 175, 225, ..., 1025 Hz
 * @rationale: 使用静态表避免运行时计算，提升查表性能
 */
static const uint16_t s_freq_table[APP_MSG_FREQ_COUNT] = {
    125,  175,  225,  275,  325,  375,  425,  475,  525,  575,
    625,  675,  725,  775,  825,  875,  925,  975,  1025
};

/* ============================================================================ */
/* 公开 API 实现                                                                 */
/* ============================================================================ */

int32_t app_msg_freq_get_array(const uint16_t **p_freq_array, uint8_t *p_count)
{
    /* 参数检查 */
    if (p_freq_array == NULL || p_count == NULL)
    {
        return -1;
    }
    
    /* 返回频点表指针和个数 */
    *p_freq_array = s_freq_table;
    *p_count = APP_MSG_FREQ_COUNT;
    
    return 0;
}

uint16_t app_msg_freq_from_index(uint8_t index)
{
    /* 索引范围检查 */
    if (index >= APP_MSG_FREQ_COUNT)
    {
        return 0;
    }
    
    return s_freq_table[index];
}

uint8_t app_msg_freq_to_index(uint16_t f_hz)
{
    /* 下界检查：频率不能低于 (起始频点 - 容差) */
    if (f_hz < (uint16_t)(APP_MSG_FREQ_START - APP_MSG_FREQ_TOLERANCE))
    {
        return APP_MSG_FREQ_INVALID_INDEX;
    }

    /* 计算相对起始频点的偏移 */
    int32_t delta = (int32_t)f_hz - (int32_t)APP_MSG_FREQ_START;
    if (delta < 0)
    {
        delta = 0;
    }

    /* 计算最近的表内频点索引 */
    int32_t idx    = delta / (int32_t)APP_MSG_FREQ_STEP;
    int32_t offset = delta % (int32_t)APP_MSG_FREQ_STEP;

    /* 四舍五入：如果余数 > 步进的一半，则进到下一个频点 */
    if (offset > (int32_t)(APP_MSG_FREQ_STEP / 2))
    {
        ++idx;
        offset = (int32_t)APP_MSG_FREQ_STEP - offset;
    }

    /* 索引范围检查 */
    if (idx < 0 || idx >= (int32_t)APP_MSG_FREQ_COUNT)
    {
        return APP_MSG_FREQ_INVALID_INDEX;
    }

    /* 容差范围检查：偏移必须在容差范围内 */
    if (offset > (int32_t)APP_MSG_FREQ_TOLERANCE)
    {
        return APP_MSG_FREQ_INVALID_INDEX;
    }

    return (uint8_t)idx;
}

bool app_msg_freq_is_valid(uint16_t f_hz)
{
    /* 校验频点是否在有效频点表内 */
    return (app_msg_freq_to_index(f_hz) != APP_MSG_FREQ_INVALID_INDEX);
}
