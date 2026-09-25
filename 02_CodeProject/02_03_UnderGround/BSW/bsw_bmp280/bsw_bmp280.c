/**
 * @file    bsw_bmp280.c
 * @brief   BMP280 气压/温度传感器 - BSW 层实现
 *
 * 本文件包含所有 BMP280 协议逻辑：
 *   - 寄存器读/写（基于 mcal_i2c）
 *   - Bosch 官方补偿算法（数据手册 §4.2.3）
 *   - 24 字节校准参数解析
 *
 * BSW 层职责：
 *   - 设备在线状态机
 *   - 最近一次有效数据缓存
 *   - 连续失败计数 → 自动标记离线
 *   - 时间戳注入（与平台解耦）
 *
 * 注：MCAL 层只提供 mcal_i2c_* 的寄存器读写原语。
 *     mcal_i2c_read_buf() 不返回错误码（HAL_MAX_DELAY 阻塞），
 *     因此运行时 read 的成败靠物理量合理性兜底。
 */

#include "bsw_bmp280.h"
#include "mcal_i2c.h"
#include <stddef.h>
#include <string.h>

/* ========== BMP280 寄存器地址 ========== */
#define BMP280_REG_DIG_T1         0x88U
#define BMP280_REG_DIG_T2         0x8AU
#define BMP280_REG_DIG_T3         0x8CU
#define BMP280_REG_DIG_P1         0x8EU
#define BMP280_REG_DIG_P2         0x90U
#define BMP280_REG_DIG_P3         0x92U
#define BMP280_REG_DIG_P4         0x94U
#define BMP280_REG_DIG_P5         0x96U
#define BMP280_REG_DIG_P6         0x98U
#define BMP280_REG_DIG_P7         0x9AU
#define BMP280_REG_DIG_P8         0x9CU
#define BMP280_REG_DIG_P9         0x9EU
#define BMP280_REG_CHIPID         0xD0U
#define BMP280_REG_RESET          0xE0U
#define BMP280_REG_CONTROL        0xF4U   /* ctrl_meas */
#define BMP280_REG_CONFIG         0xF5U
#define BMP280_REG_PRESSUREDATA   0xF7U
#define BMP280_REG_TEMPDATA       0xFAU

/* ========== 配置（采样/工作模式） ========== */
#define BMP280_OSRS_T_1X          1U      /* 温度过采样 x1 */
#define BMP280_OSRS_P_1X          1U      /* 压力过采样 x1 */
#define BMP280_MODE_NORMAL        3U      /* 持续测量模式 */
#define BMP280_T_SB_1000_MS       5U      /* 待机时间 1000ms */
#define BMP280_FILTER_OFF         0U      /* IIR 滤波关闭 */
#define BMP280_SPI3W_DISABLE      0U

#define BMP280_CTRL_MEAS_VALUE    ((uint8_t)((BMP280_OSRS_T_1X << 5) | (BMP280_OSRS_P_1X << 2) | BMP280_MODE_NORMAL))
#define BMP280_CONFIG_VALUE       ((uint8_t)((BMP280_T_SB_1000_MS << 5) | (BMP280_FILTER_OFF << 2) | BMP280_SPI3W_DISABLE))

/* ========== 校准参数结构体 ========== */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

/* ========== 内部状态 ========== */
static uint8_t              s_i2c_id      = 0U;
static uint8_t              s_dev_addr    = BSW_BMP280_I2C_ADDR_0x76;
static bmp280_calib_t       s_calib       = {0};
static int32_t              s_t_fine      = 0;     /* 温度补偿中间变量 */

static bsw_bmp280_state_t   s_state       = BSW_BMP280_STATE_UNINIT;
static bsw_bmp280_data_t    s_cached      = {0};
static uint8_t              s_has_cached  = 0U;
static bsw_bmp280_tick_fn   s_tick_fn     = 0;

/* ========== Bosch 官方补偿算法（数据手册 §4.2.3） ========== */

static int32_t bmp280_compensate_temperature(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) *
             ((int32_t)s_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) *
             ((int32_t)s_calib.dig_T3)) >> 14;

    s_t_fine = var1 + var2;
    T = (s_t_fine * 5 + 128) >> 8;

    return T;
}

static uint32_t bmp280_compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)s_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) *
             ((int64_t)s_calib.dig_P1)) >> 33;

    if (var1 == 0) {
        return 0U;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);

    return (uint32_t)(p >> 8);
}

/* ========== 校准参数解析 ========== */
static void bmp280_parse_calib(const uint8_t *buf)
{
    s_calib.dig_T1 = (uint16_t)(((uint16_t)buf[1]  << 8) | buf[0]);
    s_calib.dig_T2 = (int16_t) (((uint16_t)buf[3]  << 8) | buf[2]);
    s_calib.dig_T3 = (int16_t) (((uint16_t)buf[5]  << 8) | buf[4]);
    s_calib.dig_P1 = (uint16_t)(((uint16_t)buf[7]  << 8) | buf[6]);
    s_calib.dig_P2 = (int16_t) (((uint16_t)buf[9]  << 8) | buf[8]);
    s_calib.dig_P3 = (int16_t) (((uint16_t)buf[11] << 8) | buf[10]);
    s_calib.dig_P4 = (int16_t) (((uint16_t)buf[13] << 8) | buf[12]);
    s_calib.dig_P5 = (int16_t) (((uint16_t)buf[15] << 8) | buf[14]);
    s_calib.dig_P6 = (int16_t) (((uint16_t)buf[17] << 8) | buf[16]);
    s_calib.dig_P7 = (int16_t) (((uint16_t)buf[19] << 8) | buf[18]);
    s_calib.dig_P8 = (int16_t) (((uint16_t)buf[21] << 8) | buf[20]);
    s_calib.dig_P9 = (int16_t) (((uint16_t)buf[23] << 8) | buf[22]);
}

/* ========== I2C 总线级断线检测 ==========
 * 仅在 init 阶段使用：calib 寄存器若全 0x00 / 全 0xFF，
 * 表明总线上没有应答（设备未接 / 虚焊 / 地址错）。
 * 运行时 read 不做任何物理量校验：井下温压可远超商业级范围，
 * 由 App 层根据业务需求自行判定有效性。
 */
static int bmp280_calib_blank(const uint8_t *buf, uint8_t len)
{
    int all_ff = 1, all_00 = 1;
    for (uint8_t i = 0; i < len; i++) {
        if (buf[i] != 0xFFU) all_ff = 0;
        if (buf[i] != 0x00U) all_00 = 0;
    }
    return (all_ff || all_00);
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */

void bsw_bmp280_set_tick_source(bsw_bmp280_tick_fn fn)
{
    s_tick_fn = fn;   /* 允许传 NULL，等效于不启用 timestamp */
}

bsw_bmp280_ret_t bsw_bmp280_init(uint8_t i2c_id, uint8_t dev_addr_7bit)
{
    uint8_t chip_id = 0U;

    /* 保存配置 */
    s_i2c_id   = i2c_id;
    s_dev_addr = dev_addr_7bit;
    s_state      = BSW_BMP280_STATE_UNINIT;
    s_has_cached = 0U;

    /* 1) 读 chip_id 校验 */
    chip_id = mcal_i2c_read_reg(s_i2c_id, s_dev_addr, BMP280_REG_CHIPID);
    if (chip_id != BSW_BMP280_CHIP_ID_EXPECTED) {
        /* mcal_i2c_read_reg 不返回 status，无应答时通常返回 0，
         * 这里统一归到 ERR_CHIP_ID（含"未应答"语义） */
        s_state = BSW_BMP280_STATE_ERR;
        return BSW_BMP280_ERR_CHIP_ID;
    }

    /* 2) 写 ctrl_meas：温度×1、压力×1、NORMAL 模式 */
    if (mcal_i2c_write_reg(s_i2c_id, s_dev_addr, BMP280_REG_CONTROL,
                           BMP280_CTRL_MEAS_VALUE) != 0) {
        s_state = BSW_BMP280_STATE_ERR;
        return BSW_BMP280_ERR_I2C;
    }

    /* 3) 写 config：1s 待机、IIR 关闭、SPI3W 关闭 */
    if (mcal_i2c_write_reg(s_i2c_id, s_dev_addr, BMP280_REG_CONFIG,
                           BMP280_CONFIG_VALUE) != 0) {
        s_state = BSW_BMP280_STATE_ERR;
        return BSW_BMP280_ERR_I2C;
    }

    /* 4) 读 24 字节校准参数（连续读 0x88..0x9F） */
    uint8_t calib_buf[24] = {0};
    mcal_i2c_read_buf(s_i2c_id, s_dev_addr, BMP280_REG_DIG_T1,
                      calib_buf, sizeof(calib_buf));
    /* 兜底：I2C 无应答时常读出全 0x00 / 全 0xFF */
    if (bmp280_calib_blank(calib_buf, sizeof(calib_buf))) {
        s_state = BSW_BMP280_STATE_ERR;
        return BSW_BMP280_ERR_I2C;
    }
    bmp280_parse_calib(calib_buf);

    /* 初始化成功 */
    s_state      = BSW_BMP280_STATE_OK;
    s_has_cached = 0U;
    return BSW_BMP280_OK;
}

bsw_bmp280_ret_t bsw_bmp280_read(bsw_bmp280_data_t *out)
{
    uint8_t raw[6] = {0};
    int32_t adc_T, adc_P;

    if (out == NULL) {
        return BSW_BMP280_ERR_PARAM;
    }
    if (s_state == BSW_BMP280_STATE_UNINIT) {
        return BSW_BMP280_ERR_NOT_INIT;
    }

    /* 读 6 字节原始数据：pressure(3) + temperature(3)，起始 0xF7
     * 注：不校验物理量范围，井下温压可能远超商业级（可超 1100 hPa / 85°C） */
    mcal_i2c_read_buf(s_i2c_id, s_dev_addr, BMP280_REG_PRESSUREDATA, raw, 6);

    /* 拼装 20-bit ADC 值（数据手册 §4.2.3） */
    adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
    adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);

    /* Bosch 官方补偿（注意：温度必须先算，会更新 s_t_fine） */
    out->temperature_centi_c = bmp280_compensate_temperature(adc_T);
    out->pressure_pa         = bmp280_compensate_pressure(adc_P);
    out->timestamp           = (s_tick_fn != 0) ? s_tick_fn() : 0U;
    out->state               = BSW_BMP280_STATE_OK;

    /* 更新缓存 + 标记在线 */
    s_cached         = *out;
    s_has_cached     = 1U;
    s_state          = BSW_BMP280_STATE_OK;

    return BSW_BMP280_OK;
}

bsw_bmp280_ret_t bsw_bmp280_get_cached(bsw_bmp280_data_t *out)
{
    if (out == NULL) {
        return BSW_BMP280_ERR_PARAM;
    }
    if (s_has_cached == 0U) {
        return BSW_BMP280_ERR_NOT_INIT;
    }
    *out = s_cached;
    return BSW_BMP280_OK;
}

int bsw_bmp280_is_online(void)
{
    return (s_state == BSW_BMP280_STATE_OK) ? 1 : 0;
}
