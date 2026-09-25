/**
 * @file    app_sensor.h
 * @brief   传感器聚合 - APP 层
 *
 * @note    统一对外接口：
 *          - app_sensor_init()        系统启动时调用一次
 *          - app_sensor_task(now)      业务任务每 ~50ms 调一次，内部按
 *                                       APP_SENSOR_SAMPLE_PERIOD_MS 周期
 *                                       触发温度（DS18B20）+ 气压（BMP280）
 *          - app_sensor_get_data(&s)  任何业务方（FSM / 通信 / 日志）
 *                                       一次拿到完整快照
 *          - app_sensor_get_str()     调试日志用
 *
 * @note    不做物理量范围校验：井下温压可远超商业级（1100 hPa / 85°C），
 *          采集到什么数据直接缓存，调用方通过 per-field valid 位自行判定。
 *
 * @dependency  bsw_ds18b20 / bsw_bmp280 （BSW 层）
 * @dependency  bsw_log （诊断日志，可选）
 */

#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ========== 采样周期 ==========
 * 井下应用：井上不下发命令时，数据无需高频更新。
 *            1 分钟节奏既能看到温压变化趋势，又不浪费总线/能耗。
 *            井上主动下发命令读取数据时，app_sensor_get_data()
 *            永远返回当前缓存快照（不阻塞）。
 *
 * DS18B20 12-bit 单次转换 ~750ms，但 1 分钟一次的节奏下
 * 不构成瓶颈。
 */
#ifndef APP_SENSOR_SAMPLE_PERIOD_MS
#define APP_SENSOR_SAMPLE_PERIOD_MS   60000U   /* 60s = 1 分钟 */
#endif

/* ========== 传感器快照 ==========
 * 一次采样产出（温度、气压）放在同一个时间戳下：
 *   - temp_valid  = 本次 last_update 时刻温度是否成功采集
 *   - press_valid = 本次 last_update 时刻气压是否成功采集
 * 两个 valid 互相独立，任一为 0 时，对应字段保持上次值不变
 * （调用方仍可读，但应忽略 / 自决）。
 */
typedef struct {
    int16_t   temperature_centi_c;   /* ×0.01°C  范围无限制（井下） */
    uint32_t  pressure_pa;           /* Pa        范围无限制（井下） */
    uint32_t  last_update;           /* HAL_GetTick，0 = 从未采样 */
    uint8_t   temp_valid  : 1;
    uint8_t   press_valid : 1;
} app_sensor_data_t;

/* ========== API ========== */

/**
 * @brief   传感器 APP 层初始化
 * @note    系统启动调一次。清空快照，所有 valid 归 0。
 */
void app_sensor_init(void);

/**
 * @brief   传感器周期性任务
 * @note    业务任务（app_task）每 50~100ms 调一次，由本函数内部控制
 *          真实采样节奏（APP_SENSOR_SAMPLE_PERIOD_MS）。
 * @param   now   当前时刻（HAL_GetTick）
 */
void app_sensor_task(uint32_t now);

/**
 * @brief   获取当前传感器快照
 * @param   out  输出快照
 * @retval  0    成功（out 已填充；调用方查 valid 位判定可用性）
 *          <0   参数错误
 *
 * @note    调用方拿到 out 后判断：
 *            if (out.temp_valid)  use(out.temperature_centi_c);
 *            if (out.press_valid) use(out.pressure_pa);
 *          至少一个 valid 才算本帧成功。
 */
int app_sensor_get_data(app_sensor_data_t *out);

/**
 * @brief   调试用字符串（堆叠显示温度 + 气压）
 * @retval  形如 "T=25.50 C  P=101.325 kPa"（static 内部 buf，非线程安全）
 * @note    仅用于 bsw_log / 串口调试，正式通信帧不要用此格式。
 */
const char *app_sensor_get_str(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_H */
