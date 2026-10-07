/**
 * @file    bsw_freq_storage.c
 * @brief   频点配置 Flash 存储模块实现
 */

#include "bsw_freq_storage.h"
#include "stm32l4xx_hal.h"
#include <string.h>

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

/* 存储结构（8 字节对齐）*/
typedef struct {
    uint32_t magic;     /* 魔术字，用于校验数据有效性 */
    uint16_t f0_hz;     /* 频点 0 (Hz) */
    uint16_t f1_hz;     /* 频点 1 (Hz) */
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

int32_t bsw_freq_storage_save(uint16_t f0_hz, uint16_t f1_hz)
{
    if (!s_initialized) {
        return -1;
    }

    freq_config_t config;
    config.magic = FREQ_MAGIC;
    config.f0_hz = f0_hz;
    config.f1_hz = f1_hz;

    _flash_unlock();

    /* 擦除配置页 */
    if (_erase_config_page() != 0) {
        _flash_lock();
        return -1;
    }

    /* 写入配置（8 字节，一次 DOUBLEWORD 写入）*/
    uint64_t data;
    memcpy(&data, &config, sizeof(data));
    
    if (_write_doubleword(FREQ_STORAGE_ADDR, data) != 0) {
        _flash_lock();
        return -1;
    }

    _flash_lock();
    return 0;
}

int32_t bsw_freq_storage_load(uint16_t *f0_hz, uint16_t *f1_hz)
{
    if (!s_initialized || f0_hz == NULL || f1_hz == NULL) {
        return -1;
    }

    /* 直接从 Flash 读取（不需要解锁）*/
    const freq_config_t *p_config = (const freq_config_t *)FREQ_STORAGE_ADDR;

    /* 校验魔术字 */
    if (p_config->magic != FREQ_MAGIC) {
        return -1;  /* 未初始化或数据损坏 */
    }

    /* 读取频点 */
    *f0_hz = p_config->f0_hz;
    *f1_hz = p_config->f1_hz;

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
