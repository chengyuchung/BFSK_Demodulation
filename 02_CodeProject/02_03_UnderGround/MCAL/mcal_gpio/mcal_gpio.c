/**
 * @file    mcal_gpio.c
 * @brief   GPIO 驱动 - MCAL 层实现
 * @note    复用 CubeMX 已配置的 GPIO，向上提供统一引脚编号接口
 */

#include "mcal_gpio.h"

/* ========== CubeMX 引脚映射表 ========== */
/* 索引 = mcal_gpio.h 中的 GPIO_PIN_xxx 编号 */
typedef struct {
    GPIO_TypeDef *port;  /* GPIO 端口 */
    uint16_t      pin;   /* 引脚号 (GPIO_PIN_x) */
} gpio_pin_map_t;

static const gpio_pin_map_t s_gpio_map[] = {
    [GPIO_PIN_RELAY]         = {Relay_Control_GPIO_Port,      Relay_Control_Pin},      /* PA12 */
    [GPIO_PIN_AMP_EN]        = {0,                            0},                      /* 预留，BFSK 模块定义 */
    [GPIO_PIN_DS18B20]       = {DS18B20_GPIO_Port,            DS18B20_Pin},            /* PA2  */
    [GPIO_PIN_LED_STATUS]    = {LED_Control_GPIO_Port,        LED_Control_Pin},        /* PB5  */
    [GPIO_PIN_AD9833_FSYNC]  = {AD9833_FSYNC_GPIO_Port,       AD9833_FSYNC_Pin},       /* PC4  */
};

/* ========== 引脚数量 ========== */
#define GPIO_PIN_COUNT (sizeof(s_gpio_map) / sizeof(s_gpio_map[0]))

/* ========== 内部辅助 ========== */
static GPIO_PinState _to_hal_level(uint8_t level)
{
    return (level == GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* ================================================================ */
/*                           公共接口实现                            */
/* ================================================================ */

void mcal_gpio_init(void)
{
    /* CubeMX MX_GPIO_Init() 已完成所有引脚初始化，MCAL 层无需重复配置 */
}

void mcal_gpio_write(uint8_t pin, uint8_t level)
{
    if (pin >= GPIO_PIN_COUNT) return;
    if (s_gpio_map[pin].port == 0) return;  /* 未定义引脚 */

    HAL_GPIO_WritePin(s_gpio_map[pin].port,
                      s_gpio_map[pin].pin,
                      _to_hal_level(level));
}

uint8_t mcal_gpio_read(uint8_t pin)
{
    if (pin >= GPIO_PIN_COUNT) return 0;
    if (s_gpio_map[pin].port == 0) return 0;  /* 未定义引脚 */

    GPIO_PinState state = HAL_GPIO_ReadPin(s_gpio_map[pin].port,
                                           s_gpio_map[pin].pin);
    return (state == GPIO_PIN_SET) ? GPIO_HIGH : GPIO_LOW;
}

void mcal_gpio_set_output(uint8_t pin)
{
    /* DS18B20 需要动态切换输入/输出模式 */
    /* 其他引脚由 CubeMX 配置，MCAL 层不需要重复设置 */
    if (pin >= GPIO_PIN_COUNT) return;
    if (s_gpio_map[pin].port == 0) return;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = s_gpio_map[pin].pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(s_gpio_map[pin].port, &GPIO_InitStruct);
}

void mcal_gpio_set_output_od(uint8_t pin)
{
    if (pin >= GPIO_PIN_COUNT) return;
    if (s_gpio_map[pin].port == 0) return;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = s_gpio_map[pin].pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;  /* 开漏：1-Wire 总线必须 */
    GPIO_InitStruct.Pull  = GPIO_PULLUP;          /* 内部上拉，总线空闲时为高 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(s_gpio_map[pin].port, &GPIO_InitStruct);
}

void mcal_gpio_set_input(uint8_t pin)
{
    if (pin >= GPIO_PIN_COUNT) return;
    if (s_gpio_map[pin].port == 0) return;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = s_gpio_map[pin].pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;  /* DS18B20 需要上拉 */

    HAL_GPIO_Init(s_gpio_map[pin].port, &GPIO_InitStruct);
}
