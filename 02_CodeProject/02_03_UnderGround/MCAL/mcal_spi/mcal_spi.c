/**
 * @file    mcal_spi.c
 * @brief   SPI 驱动 - MCAL 层实现
 * @note    原 main.c 的 MX_SPI1_Init() 已移植到本文件
 *          - SPI1: 主模式、8 位、波特率 PCLK/32、软件 NSS（给 AD9833 用）
 */

#include "mcal_spi.h"

extern void Error_Handler(void);

/* ========== 内部句柄 ========== */
static SPI_HandleTypeDef s_hspi1;

/* ========== Getter ========== */
SPI_HandleTypeDef *mcal_spi_get_handle1(void) { return &s_hspi1; }

/* ========== HAL SPI MSP 初始化重写 ========== */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hspi == &s_hspi1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /* PA5 = SCK, PA6 = MISO, PA7 = MOSI（注：MISO 可能未使用，保留） */
        GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* ================================================================ */
/*              原 MX_SPI1_Init 内容（移植）                          */
/* ================================================================ */
static void _spi1_init(void)
{
    s_hspi1.Instance               = SPI1;
    s_hspi1.Init.Mode              = SPI_MODE_MASTER;
    s_hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    s_hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    s_hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    s_hspi1.Init.NSS               = SPI_NSS_SOFT;
    s_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    s_hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    s_hspi1.Init.CRCPolynomial     = 7;
    s_hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    s_hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;

    if (HAL_SPI_Init(&s_hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */
void mcal_spi_init(uint8_t id)
{
    switch (id) {
        case SPI_ID_1: _spi1_init(); break;
        default: break;
    }
}

uint8_t mcal_spi_transfer(uint8_t id, uint8_t tx_data)
{
    SPI_HandleTypeDef *hspi = NULL;
    switch (id) {
        case SPI_ID_1: hspi = &s_hspi1; break;
        default: return 0;
    }
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(hspi, &tx_data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

void mcal_spi_transfer_buf(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
    SPI_HandleTypeDef *hspi = NULL;
    switch (id) {
        case SPI_ID_1: hspi = &s_hspi1; break;
        default: return;
    }
    if (rx_buf) {
        HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, len, HAL_MAX_DELAY);
    } else {
        HAL_SPI_Transmit(hspi, tx_buf, len, HAL_MAX_DELAY);
    }
}
