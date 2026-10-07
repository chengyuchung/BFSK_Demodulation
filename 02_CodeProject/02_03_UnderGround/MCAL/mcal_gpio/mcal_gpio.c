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
    [GPIO_PIN_LED_LISTEN]    = {GPIOB,                        GPIO_PIN_5},             /* PB5 LED1：扫频监听态 */
    [GPIO_PIN_LED_PRELINK]   = {GPIOB,                        GPIO_PIN_4},             /* PB4 LED2：预链接态 */
    [GPIO_PIN_LED_LINK]      = {GPIOB,                        GPIO_PIN_3},             /* PB3 LED3：工作态 */
    [GPIO_PIN_LED_SCAN]      = {GPIOD,                        GPIO_PIN_2},             /* PD2 LED4：扫频模式 */
    [GPIO_PIN_LED_ERROR]     = {GPIOC,                        GPIO_PIN_12},            /* PC12 LED5：故障模式 */
    [GPIO_PIN_AD9833_FSYNC]  = {AD9833_FSYNC_GPIO_Port,       AD9833_FSYNC_Pin},       /* PC4  */
    [GPIO_PIN_COMP_EN]       = {GPIOC,                        GPIO_PIN_10},            /* PC10 比较器电路使能（直接硬编码） */
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
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* PC10（比较器使能）：CubeMX 未配置，需要手动初始化 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    
    /* PC12（LED_ERROR 故障指示灯）：CubeMX 未配置，需要手动初始化 */
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    
    /* PB3/PB4/PB5（3 个 LED）：CubeMX 可能只配置了 PB5，手动初始化所有 LED */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    
    /* PD2（LED_SCAN 扫频指示灯）：CubeMX 未配置，需要手动初始化 */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
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
