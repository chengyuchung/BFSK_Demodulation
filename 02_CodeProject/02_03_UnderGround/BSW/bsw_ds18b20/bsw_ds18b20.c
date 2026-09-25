/**
 * @file    bsw_ds18b20.c
 * @brief   DS18B20 温度传感器 - BSW 层实现
 *
 * 本文件包含所有 DS18B20 协议逻辑：
 *   - Maxim/Dallas CRC8 校验
 *   - Skip ROM / Convert T / Read Scratchpad 命令
 *   - 12bit 温度换算 (×0.01°C)
 *   - scratchpad 全 0xFF/全 0x00 断线检测
 *   - 转换完成轮询（DQ 状态）
 *
 * BSW 层职责：
 *   - 设备在线状态机
 *   - 最近一次有效温度缓存
 *   - 连续失败计数 → 标记离线
 *
 * 注：MCAL 层只提供 mcal_ow_* 的字节流原语。
 */

#include "bsw_ds18b20.h"
#include "mcal_ow.h"
#include "mcal_timer.h"

/* ========== DS18B20 命令字 ========== */
#define DS18B20_CMD_SKIP_ROM        0xCC
#define DS18B20_CMD_CONVERT_T       0x44
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE

/* ========== Scratchpad 长度 ========== */
#define DS18B20_SCRATCHPAD_LEN      9
#define DS18B20_CONVERT_TIMEOUT_MS  800   /* 12bit 默认分辨率上限 750ms */

/* ========== 配置 ========== */
#define BSW_DS18B20_MAX_CONSEC_ERR  3   /* 连续失败次数达此值认为离线 */

/* ========== Maxim/Dallas CRC8 ==========
 * 多项式: X^8 + X^5 + X^4 + 1 (0x8C 反射)
 */
static uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (uint8_t)((crc ^ inbyte) & 0x01u);
            crc >>= 1;
            if (mix) crc ^= 0x8Cu;
            inbyte >>= 1;
        }
    }
    return crc;
}

/* ========== DS18B20 协议级操作（基于 mcal_ow） ========== */

static ds18b20_ret_t ds18b20_start_convert(void)
{
    if (mcal_ow_reset() != OW_OK) return DS18B20_ERR_NO_DEVICE;
    mcal_ow_write_byte(DS18B20_CMD_SKIP_ROM);   /* 单设备，跳过 ROM */
    mcal_ow_write_byte(DS18B20_CMD_CONVERT_T);  /* 启动温度转换 */
    return DS18B20_OK;
}

/**
 * @brief   轮询等待温度转换完成
 * @note    DS18B20 转换期间 DQ 拉低，完成后释放（变高）
 *          用 read_bit 在 ~15us 处采样 DQ 判断
 */
static ds18b20_ret_t ds18b20_wait_conversion(uint32_t timeout_ms)
{
    uint32_t waited = 0;

    /* 起始稍等，避免转换刚启动就被误判为完成 */
    mcal_timer_delay_us(10);

    while (waited < timeout_ms) {
        if (mcal_ow_read_bit() != 0) {
            return DS18B20_OK;   /* DQ 被释放=高 → 转换完成 */
        }
        mcal_timer_delay_ms(10);
        waited += 10;
    }

    return DS18B20_ERR_TIMEOUT;
}

static ds18b20_ret_t ds18b20_read_result(int16_t *temp_out)
{
    uint8_t scratch[DS18B20_SCRATCHPAD_LEN];
    uint8_t crc_calc;
    int16_t raw;

    if (temp_out == NULL) return DS18B20_ERR_READ;

    if (mcal_ow_reset() != OW_OK) return DS18B20_ERR_NO_DEVICE;

    mcal_ow_write_byte(DS18B20_CMD_SKIP_ROM);
    mcal_ow_write_byte(DS18B20_CMD_READ_SCRATCHPAD);

    for (uint8_t i = 0; i < DS18B20_SCRATCHPAD_LEN; i++) {
        scratch[i] = mcal_ow_read_byte();
    }

    /* 断线常读出全 0xFF 或全 0x00，提前识别 */
    uint8_t all_ff = 1, all_00 = 1;
    for (uint8_t i = 0; i < DS18B20_SCRATCHPAD_LEN; i++) {
        if (scratch[i] != 0xFF) all_ff = 0;
        if (scratch[i] != 0x00) all_00 = 0;
    }
    if (all_ff || all_00) return DS18B20_ERR_READ;

    /* CRC8 校验（前 8 字节） */
    crc_calc = ds18b20_crc8(scratch, 8);
    if (crc_calc != scratch[8]) return DS18B20_ERR_CRC;

    /* LSB=温度低字节，MSB=温度高字节（符号扩展） */
    raw = (int16_t)((uint16_t)scratch[1] << 8) | scratch[0];

    /* 12bit 分辨率: 1 LSB = 1/16 °C → 换算成 ×100 整数 */
    *temp_out = (int16_t)((int32_t)raw * 100 / 16);

    return DS18B20_OK;
}

/* ========== 状态机 + 缓存 ========== */
static bsw_ds18b20_state_t s_state       = BSW_DS18B20_UNINIT;
static bsw_ds18b20_data_t  s_cached      = {0};
static uint8_t             s_consec_err  = 0;
static uint8_t             s_has_cached  = 0;  /* 是否曾成功读过一次 */

/* 时间戳来源（由上层 App 注入，参见 bsw_ds18b20_set_tick_source） */
static bsw_ds18b20_tick_fn s_tick_fn     = 0;

static inline void on_error(void)
{
    if (++s_consec_err >= BSW_DS18B20_MAX_CONSEC_ERR) {
        s_state = BSW_DS18B20_ERR;
    }
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */

void bsw_ds18b20_set_tick_source(bsw_ds18b20_tick_fn fn)
{
    s_tick_fn = fn;   /* 允许传 NULL，等效于不启用 timestamp */
}

int bsw_ds18b20_init(void)
{
    if (mcal_ow_reset() != OW_OK) {
        s_state = BSW_DS18B20_ERR;
        return DS18B20_ERR_NO_DEVICE;
    }

    s_state       = BSW_DS18B20_OK;
    s_consec_err  = 0;
    return DS18B20_OK;
}

int bsw_ds18b20_is_online(void)
{
    return (s_state == BSW_DS18B20_OK) ? 1 : 0;
}

int bsw_ds18b20_trigger(void)
{
    if (s_state != BSW_DS18B20_OK) return -1;

    ds18b20_ret_t ret = ds18b20_start_convert();
    if (ret != DS18B20_OK) {
        on_error();
        return (int)ret;
    }
    return DS18B20_OK;
}

int bsw_ds18b20_get_result(bsw_ds18b20_data_t *data)
{
    ds18b20_ret_t ret;
    int16_t        raw = 0;

    if (data == NULL) return -1;
    if (s_state != BSW_DS18B20_OK) return -1;

    /* 启动转换 */
    ret = ds18b20_start_convert();
    if (ret != DS18B20_OK) {
        on_error();
        return (int)ret;
    }

    /* 轮询等待（自适应时长，最长 800ms） */
    ret = ds18b20_wait_conversion(DS18B20_CONVERT_TIMEOUT_MS);
    if (ret != DS18B20_OK) {
        on_error();
        return (int)ret;
    }

    /* 读取结果（CRC 校验 + 全 0xFF/0x00 检测） */
    ret = ds18b20_read_result(&raw);
    if (ret != DS18B20_OK) {
        on_error();
        return (int)ret;
    }

    /* 成功：清错误计数 + 更新缓存 */
    s_consec_err         = 0;
    s_cached.raw_value   = raw;
    s_cached.timestamp   = (s_tick_fn != 0) ? s_tick_fn() : 0U;
    s_cached.state       = BSW_DS18B20_OK;
    s_has_cached         = 1;

    data->raw_value = s_cached.raw_value;
    data->timestamp = s_cached.timestamp;
    data->state     = s_cached.state;

    return DS18B20_OK;
}

int bsw_ds18b20_get_cached(bsw_ds18b20_data_t *data)
{
    if (data == NULL) return -1;
    if (s_has_cached == 0) return -1;

    data->raw_value = s_cached.raw_value;
    data->timestamp = s_cached.timestamp;
    data->state     = s_cached.state;

    return DS18B20_OK;
}
