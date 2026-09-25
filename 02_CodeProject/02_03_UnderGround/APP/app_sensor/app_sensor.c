/**
 * @file    app_sensor.c
 * @brief   传感器聚合 - APP 层实现
 *
 * 职责：
 *   - 周期性触发 DS18B20 测温 + 读取 BMP280 气压
 *   - 维护最新快照（温度 + 气压 + 时间戳 + per-field valid）
 *   - 对外提供单个 get_data() 快照接口
 *
 * 注：
 *   - BSW 层的 init 由 app_main.c 完成（依赖 MCAL 初始化顺序）
 *   - 本模块不做物理量范围校验（井下温压可能远超商业级）
 *   - per-field valid：温度 / 气压 任意一方失败不影响另一方缓存
 */

#include "app_sensor.h"
#include "bsw_ds18b20.h"
#include "bsw_bmp280.h"
#include "bsw_log.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* ========== 内部状态 ========== */
static app_sensor_data_t s_data       = {0};   /* 当前快照（init 后全 0） */
static uint32_t          s_next_due   = 0U;    /* 下次采样时刻（HAL_GetTick）*/

/* ========== 采样 ========== */

/**
 * @brief   执行一次完整采集：温度 + 气压
 * @note    per-field valid：
 *              - 温度 BSW 失败：temp_valid = 0，温度字段保留旧值
 *              - 气压 BSW 失败：press_valid = 0，气压字段保留旧值
 *            不互相影响。
 */
static void do_collect(void)
{
    /* —— 温度：DS18B20 —— */
    bsw_ds18b20_data_t t = {0};
    if (bsw_ds18b20_get_result(&t) == 0) {
        s_data.temperature_centi_c = t.raw_value;
        s_data.temp_valid          = 1U;
    } else {
        s_data.temp_valid          = 0U;
    }

    /* —— 气压：BMP280 —— */
    bsw_bmp280_data_t p = {0};
    if (bsw_bmp280_read(&p) == BSW_BMP280_OK) {
        s_data.pressure_pa = p.pressure_pa;
        s_data.press_valid = 1U;
    } else {
        s_data.press_valid = 0U;
    }
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */

void app_sensor_init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    s_next_due = 0U;   /* 第一次 task 调用立即采样，避免启动后等 5s */
}

void app_sensor_task(uint32_t now)
{
    /* 周期未到，不做事；用减法防止 now 回绕（差异超过 int32 范围时也安全） */
    if ((int32_t)(now - s_next_due) < 0) {
        return;
    }
    s_next_due = now + APP_SENSOR_SAMPLE_PERIOD_MS;

    /* 记录采样时刻（哪怕两边都失败，也算"采了一帧"） */
    s_data.last_update = now;

    do_collect();

    /* 诊断日志已关闭（井下场景无人串口观察，省 printf 栈消耗）。
     * 调试期如需观察温压，可在上位机/井上请求命令到达后，由通信模块
     * 调用 app_sensor_get_str() 拿到格式化字符串再走另一条输出路径，
     * 这样只在"被查询"时才付一次 printf 栈代价。 */
    // bsw_log("app_sensor: %s\r\n", app_sensor_get_str());
}

int app_sensor_get_data(app_sensor_data_t *out)
{
    if (out == NULL) {
        return -1;
    }
    *out = s_data;
    return 0;
}

const char *app_sensor_get_str(void)
{
    static char buf[48];

    /* 温度格式化（绝对值拆分避免负号 + %u 不兼容） */
    int16_t raw_t = s_data.temperature_centi_c;
    int16_t abs_t = (raw_t < 0) ? (int16_t)(-raw_t) : raw_t;
    int     t_int = abs_t / 100;
    int     t_dec = abs_t % 100;

    /* 气压单位：Pa → kPa（保留 3 位小数 = Pa 精度） */
    uint32_t p_pa = s_data.pressure_pa;
    uint32_t p_kpa_int = p_pa / 1000U;
    uint32_t p_kpa_dec = p_pa % 1000U;

    snprintf(buf, sizeof(buf),
             "T=%s%d.%02d C  P=%lu.%03lu kPa%s%s",
             (raw_t < 0) ? "-" : "",
             t_int, t_dec,
             (unsigned long)p_kpa_int, (unsigned long)p_kpa_dec,
             s_data.temp_valid  ? "" : " (T-NA)",
             s_data.press_valid ? "" : " (P-NA)");

    return buf;
}
