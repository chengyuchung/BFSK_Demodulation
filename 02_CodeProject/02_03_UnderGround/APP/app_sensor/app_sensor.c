/**
 * @file    app_sensor.c
 * @brief   传感器管理 - APP 层实现
 */

#include "app_sensor.h"
#include <stdio.h>
#include <string.h>

/* ========== 静态变量 ========== */
static uint32_t       s_last_sample_time = 0;  /* 上次采样时刻 */
static app_sensor_data_t s_data = {0};          /* 当前传感器数据 */
static app_sensor_cb_t   s_callback = NULL;     /* 回调函数 */

/* ========== 内部辅助 ========== */

/**
 * @brief   检查温度值是否在合理范围内
 */
static int _is_temp_valid(int16_t raw)
{
    return (raw >= APP_SENSOR_TEMP_MIN && raw <= APP_SENSOR_TEMP_MAX);
}

/**
 * @brief   格式化温度为字符串
 */
static void _format_temp(int16_t raw, char *buf, size_t bufsize)
{
    if (!_is_temp_valid(raw)) {
        snprintf(buf, bufsize, "--");
        return;
    }

    int16_t abs_val = (raw < 0) ? (-raw) : raw;
    int16_t deg  = abs_val / 100;
    int16_t cent = abs_val % 100;

    if (raw < 0) {
        snprintf(buf, bufsize, "-%d.%02d", deg, cent);
    } else {
        snprintf(buf, bufsize, "%d.%02d", deg, cent);
    }
}

/* ================================================================ */
/*                           公共接口实现                            */
/* ================================================================ */

void app_sensor_init(void)
{
    /* 初始化 BSW 层（检测 DS18B20 设备） */
    int ret = bsw_ds18b20_init();

    if (ret == 0) {
        s_data.online = 1;
        s_data.valid  = 0;
    } else {
        s_data.online = 0;
        s_data.valid  = 0;
    }

    s_last_sample_time = 0;
    s_callback = NULL;
}

void app_sensor_task(uint32_t now)
{
    /* 设备离线时不采样 */
    if (!s_data.online) {
        /* 尝试重新检测设备 */
        if (bsw_ds18b20_is_online()) {
            s_data.online = 1;
        }
        return;
    }

    /* 判断是否到达采样周期 */
    if ((now - s_last_sample_time) < APP_SENSOR_SAMPLE_PERIOD_MS) {
        return;
    }

    s_last_sample_time = now;

    /* 触发一次温度采样 */
    bsw_ds18b20_data_t bsw_data;
    int ret = bsw_ds18b20_get_result(&bsw_data);

    if (ret == 0 && _is_temp_valid(bsw_data.raw_value)) {
        s_data.temperature  = bsw_data.raw_value;
        s_data.last_update  = now;
        s_data.valid       = 1;

        /* 触发回调通知 */
        if (s_callback != NULL) {
            s_callback(bsw_data.raw_value);
        }
    } else {
        /* 采样失败，标记数据无效 */
        s_data.valid = 0;
    }
}

int app_sensor_get_data(app_sensor_data_t *data)
{
    if (data == NULL) return -1;

    data->temperature  = s_data.temperature;
    data->last_update  = s_data.last_update;
    data->valid        = s_data.valid;
    data->online        = s_data.online;

    return s_data.valid ? 0 : -1;
}

int16_t app_sensor_get_raw(void)
{
    return s_data.valid ? s_data.temperature : 0;
}

const char *app_sensor_get_str(void)
{
    static char buf[16];

    if (!s_data.valid) {
        return "--";
    }

    _format_temp(s_data.temperature, buf, sizeof(buf));
    return buf;
}

void app_sensor_register_callback(app_sensor_cb_t cb)
{
    s_callback = cb;
}
