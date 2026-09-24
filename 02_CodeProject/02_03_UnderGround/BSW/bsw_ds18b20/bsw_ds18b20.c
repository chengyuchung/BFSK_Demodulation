/**
 * @file    bsw_ds18b20.c
 * @brief   DS18B20 传感器 - BSW 层实现
 *
 * 关键特性：
 *   - 使用 MCAL 层的 wait_conversion 轮询（非固定延时）
 *   - 区分 CRC 错误/无设备/超时，给上层更精确的状态
 *   - 错误计数器：连续失败 N 次标记离线
 */

#include "bsw_ds18b20.h"
#include "mcal_timer.h"

/* ========== 配置 ========== */
#define BSW_DS18B20_MAX_CONSEC_ERR    3   /* 连续失败次数达此值认为离线 */

/* ========== 静态变量 ========== */
static bsw_ds18b20_state_t s_state = BSW_DS18B20_UNINIT;
static bsw_ds18b20_data_t  s_cached = {0};
static uint8_t             s_consec_err = 0;   /* 连续错误计数 */

/* ================================================================ */
/*                           公共接口实现                            */
/* ================================================================ */

int bsw_ds18b20_init(void)
{
    ds18b20_ret_t ret = mcal_ds18b20_init();

    if (ret == DS18B20_OK) {
        s_state = BSW_DS18B20_OK;
        s_consec_err = 0;
        return 0;
    }

    s_state = BSW_DS18B20_ERR;
    return (int)ret;
}

int bsw_ds18b20_is_online(void)
{
    return (s_state == BSW_DS18B20_OK) ? 1 : 0;
}

int bsw_ds18b20_trigger(void)
{
    if (s_state != BSW_DS18B20_OK) return -1;

    ds18b20_ret_t ret = mcal_ds18b20_start_convert();
    return (ret == DS18B20_OK) ? 0 : (int)ret;
}

int bsw_ds18b20_get_result(bsw_ds18b20_data_t *data)
{
    ds18b20_ret_t ret;
    int16_t raw = 0;

    if (data == NULL) return -1;
    if (s_state != BSW_DS18B20_OK) return -1;

    /* 启动转换 */
    if (mcal_ds18b20_start_convert() != DS18B20_OK) {
        s_consec_err++;
        if (s_consec_err >= BSW_DS18B20_MAX_CONSEC_ERR) s_state = BSW_DS18B20_ERR;
        return -1;
    }

    /* 轮询等待转换完成（自适应时长，最长 800ms） */
    ret = mcal_ds18b20_wait_conversion(800);
    if (ret != DS18B20_OK) {
        s_consec_err++;
        if (s_consec_err >= BSW_DS18B20_MAX_CONSEC_ERR) s_state = BSW_DS18B20_ERR;
        return (int)ret;
    }

    /* 读取结果（带 CRC 校验） */
    ret = mcal_ds18b20_read_result(&raw);
    if (ret != DS18B20_OK) {
        s_consec_err++;
        if (s_consec_err >= BSW_DS18B20_MAX_CONSEC_ERR) s_state = BSW_DS18B20_ERR;
        return (int)ret;
    }

    /* 成功：清除错误计数 */
    s_consec_err = 0;

    /* 更新缓存 */
    s_cached.raw_value  = raw;
    s_cached.timestamp  = HAL_GetTick();
    s_cached.state      = BSW_DS18B20_OK;

    /* 输出 */
    data->raw_value  = s_cached.raw_value;
    data->timestamp  = s_cached.timestamp;
    data->state      = s_cached.state;

    return 0;
}

int bsw_ds18b20_get_cached(bsw_ds18b20_data_t *data)
{
    if (data == NULL) return -1;
    if (s_cached.timestamp == 0) return -1;

    data->raw_value = s_cached.raw_value;
    data->timestamp = s_cached.timestamp;
    data->state     = s_cached.state;

    return 0;
}
