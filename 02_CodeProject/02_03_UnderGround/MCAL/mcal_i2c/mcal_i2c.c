/**
 * @file    mcal_i2c.c
 * @brief   I2C 驱动 - MCAL 层实现
 * @note    原 main.c 的 MX_I2C1_Init() 已移植到本文件
 *          - I2C1: 7-bit 地址模式，时序 0x10D19CE4（BMP280 用）
 *          - 引脚: PB8=SCL, PB9=SDA
 */

#include "mcal_i2c.h"

extern void Error_Handler(void);

/* ========== 内部句柄 ========== */
static I2C_HandleTypeDef s_hi2c1;

/* ========== Getter ========== */
I2C_HandleTypeDef *mcal_i2c_get_handle1(void) { return &s_hi2c1; }

/* ========== HAL I2C MSP 重写 ========== */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hi2c == &s_hi2c1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   /* PB8=SCL, PB9=SDA */
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;              /* 外部上拉 */
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* ================================================================ */
/*              原 MX_I2C1_Init 内容（移植）                          */
/* ================================================================ */
static void _i2c1_init(void)
{
    s_hi2c1.Instance              = I2C1;
    s_hi2c1.Init.Timing           = 0x10D19CE4;
    s_hi2c1.Init.OwnAddress1      = 0;
    s_hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    s_hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    s_hi2c1.Init.OwnAddress2      = 0;
    s_hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    s_hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    s_hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&s_hi2c1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&s_hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&s_hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */
void mcal_i2c_init(uint8_t id)
{
    switch (id) {
        case I2C_ID_1: _i2c1_init(); break;
        default: break;
    }
}

int mcal_i2c_write_reg(uint8_t id, uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    I2C_HandleTypeDef *hi2c = NULL;
    switch (id) {
        case I2C_ID_1: hi2c = &s_hi2c1; break;
        default: return -1;
    }
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(hi2c, dev_addr << 1, reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             &data, 1, HAL_MAX_DELAY);
    return (st == HAL_OK) ? 0 : -1;
}

uint8_t mcal_i2c_read_reg(uint8_t id, uint8_t dev_addr, uint8_t reg)
{
    I2C_HandleTypeDef *hi2c = NULL;
    switch (id) {
        case I2C_ID_1: hi2c = &s_hi2c1; break;
        default: return 0;
    }
    uint8_t val = 0;
    HAL_I2C_Mem_Read(hi2c, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                     &val, 1, HAL_MAX_DELAY);
    return val;
}

void mcal_i2c_read_buf(uint8_t id, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = NULL;
    switch (id) {
        case I2C_ID_1: hi2c = &s_hi2c1; break;
        default: return;
    }
    HAL_I2C_Mem_Read(hi2c, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                     buf, len, HAL_MAX_DELAY);
}
