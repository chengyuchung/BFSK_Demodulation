/**
 * @file    bsw_freq_storage.c
 * @brief   频点配置 Flash 存储模块实现
 *
 * V2 改动：存储结构从 8 字节扩展为 16 字节，同时保存上行 + 下行两对频点
 */

#include "bsw_freq_storage.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdbool.h>

/* ========== Flash 存储配置 ========== */

/* STM32L476 有 1MB Flash (0x08000000 ~ 0x080FFFFF)
 * 每页 2KB，共 512 页
 * 使用最后一页（Page 511）作为配置存储区
 */
#define FREQ_STORAGE_PAGE_SIZE  2048U
#define FREQ_STORAGE_BASE_ADDR  0x08000000U
#define FREQ_STORAGE_TOTAL_SIZE (1024U * 1024U)
#define FREQ_STORAGE_PAGE_NUM   511U
#define FREQ_STORAGE_ADDR       (FREQ_STORAGE_BASE_ADDR + FREQ_STORAGE_TOTAL_SIZE - FREQ_STORAGE_PAGE_SIZE)

/* 魔术字 "FRQQ" (FReQuency) */
#define FREQ_MAGIC              0x46525151U

/* 存储结构（16 字节，8 字节对齐） */
typedef struct {
    uint32_t magic;           /* 魔术字，用于校验数据有效性 */
    uint16_t uplink_f0_hz;    /* 上行频点 0 (Hz) */
    uint16_t uplink_f1_hz;    /* 上行频点 1 (Hz) */
    uint16_t downlink_f0_hz;  /* 下行频点 0 (Hz) */
    uint16_t downlink_f1_hz;  /* 下行频点 1 (Hz) */
    uint32_t reserved;        /* 保留（8 字节对齐，便于后续扩展） */
} freq_config_t;

/* ========== 静态变量 ========== */
static uint8_t s_initialized = 0;

/* ========== 内部函数 ========== */

/**
 * @brief Flash 解锁
 */
static void _flash_unlock(void)
{
    HAL_FLASH_Unlock();
}

/**
 * @brief Flash 上锁
 */
static void _flash_lock(void)
{
    HAL_FLASH_Lock();
}

/**
 * @brief 擦除配置页
 * @return 0 成功，-1 失败
 */
static int32_t _erase_config_page(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks     = FLASH_BANK_1;  /* STM32L476 只有一个 Bank */
    erase_init.Page      = FREQ_STORAGE_PAGE_NUM;
    erase_init.NbPages   = 1;

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        return -1;
    }

    if (page_error != 0xFFFFFFFFU) {
        return -1;  /* 擦除失败 */
    }

    return 0;
}

/**
 * @brief 写入双字（64-bit）到 Flash
 * @param addr    目标地址（必须 8 字节对齐）
 * @param data    64-bit 数据
 * @return 0 成功，-1 失败
 */
static int32_t _write_doubleword(uint32_t addr, uint64_t data)
{
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data) != HAL_OK) {
        return -1;
    }
    return 0;
}

/* ========== 公共函数 ========== */

void bsw_freq_storage_init(void)
{
    s_initialized = 1;
}

int32_t bsw_freq_storage_save_all(uint16_t uplink_f0_hz,
                                  uint16_t uplink_f1_hz,
                                  uint16_t downlink_f0_hz,
                                  uint16_t downlink_f1_hz)
{
    if (!s_initialized) {
        return -1;
    }

    freq_config_t config;
    config.magic          = FREQ_MAGIC;
    config.uplink_f0_hz   = uplink_f0_hz;
    config.uplink_f1_hz   = uplink_f1_hz;
    config.downlink_f0_hz = downlink_f0_hz;
    config.downlink_f1_hz = downlink_f1_hz;
    config.reserved       = 0;

    _flash_unlock();

    /* 擦除配置页 */
    if (_erase_config_page() != 0) {
        _flash_lock();
        return -1;
    }

    /* 写入配置：16 字节分 2 次 DOUBLEWORD 写入
     * [0-7]:   magic + uplink_f0 + uplink_f1
     * [8-15]:  downlink_f0 + downlink_f1 + reserved
     */
    uint64_t data0, data1;
    memcpy(&data0, &config, sizeof(data0));
    memcpy(&data1, ((const uint8_t *)&config) + 8, sizeof(data1));

    if (_write_doubleword(FREQ_STORAGE_ADDR, data0) != 0) {
        _flash_lock();
        return -1;
    }

    if (_write_doubleword(FREQ_STORAGE_ADDR + 8U, data1) != 0) {
        _flash_lock();
        return -1;
    }

    _flash_lock();
    return 0;
}

int32_t bsw_freq_storage_load_all(uint16_t *uplink_f0_hz,
                                  uint16_t *uplink_f1_hz,
                                  uint16_t *downlink_f0_hz,
                                  uint16_t *downlink_f1_hz)
{
    if (!s_initialized
        || uplink_f0_hz == NULL || uplink_f1_hz == NULL
        || downlink_f0_hz == NULL || downlink_f1_hz == NULL) {
        return -1;
    }

    /* 默认输出 0（频点无效） */
    *uplink_f0_hz   = 0;
    *uplink_f1_hz   = 0;
    *downlink_f0_hz = 0;
    *downlink_f1_hz = 0;

    /* 直接从 Flash 读取（不需要解锁） */
    const freq_config_t *p_config = (const freq_config_t *)FREQ_STORAGE_ADDR;

    /* 校验魔术字 */
    if (p_config->magic != FREQ_MAGIC) {
        return -1;  /* 未初始化或数据损坏（输出已为 0） */
    }

    /* 读取频点 */
    uint16_t uf0 = p_config->uplink_f0_hz;
    uint16_t uf1 = p_config->uplink_f1_hz;
    uint16_t df0 = p_config->downlink_f0_hz;
    uint16_t df1 = p_config->downlink_f1_hz;

    /* 合理性校验：防止读出垃圾数据（Flash 未初始化时可能是 0xFFFF）
     * 
     * 检查项：
     *   1. 非零（0 表示 FREQ_NOT_LOCKED）
     *   2. 非 0xFFFF（Flash 擦除后默认值）
     *   3. 范围合理（100 ~ 2000 Hz，覆盖所有可能的扫频范围）
     * 
     * 注意：这里只做基础合理性检查，不校验是否在频点表内
     *      （频点表校验由 APP 层负责，避免 BSW 层耦合业务逻辑）
     */
    #define FREQ_MIN_REASONABLE  100u
    #define FREQ_MAX_REASONABLE  2000u

    bool uf0_valid = (uf0 != 0 && uf0 != 0xFFFF 
                      && uf0 >= FREQ_MIN_REASONABLE && uf0 <= FREQ_MAX_REASONABLE);
    bool uf1_valid = (uf1 != 0 && uf1 != 0xFFFF 
                      && uf1 >= FREQ_MIN_REASONABLE && uf1 <= FREQ_MAX_REASONABLE);
    bool df0_valid = (df0 != 0 && df0 != 0xFFFF 
                      && df0 >= FREQ_MIN_REASONABLE && df0 <= FREQ_MAX_REASONABLE);
    bool df1_valid = (df1 != 0 && df1 != 0xFFFF 
                      && df1 >= FREQ_MIN_REASONABLE && df1 <= FREQ_MAX_REASONABLE);

    /* 只有频点对两个都有效时才输出，否则保持输出为 0 */
    if (uf0_valid && uf1_valid) {
        *uplink_f0_hz = uf0;
        *uplink_f1_hz = uf1;
    }

    if (df0_valid && df1_valid) {
        *downlink_f0_hz = df0;
        *downlink_f1_hz = df1;
    }

    return 0;
}

int32_t bsw_freq_storage_clear(void)
{
    if (!s_initialized) {
        return -1;
    }

    _flash_unlock();

    int32_t ret = _erase_config_page();

    _flash_lock();

    return ret;
}
