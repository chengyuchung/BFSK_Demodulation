/* app_main.c - 井下节点 APP 层入口
 *
 * 调用顺序：
 *   mcal_clock_init()     ← 系统时钟
 *   mcal_gpio_init()      ← GPIO
 *   mcal_dma_init()       ← DMA
 *   mcal_adc_init()       ← ADC1 + ADC2
 *   mcal_timer_init()     ← TIM1/2/6
 *   mcal_spi_init()       ← SPI1（AD9833）
 *   mcal_uart_init()      ← USART1（调试串口）
 *   mcal_i2c_init()       ← I2C1（BMP280）
 *   bsw_xxx_init()        ← BSW 协议层
 *   app_xxx_init()        ← APP 业务层
 *   app_task_create_all() ← FreeRTOS 任务创建
 */

#include "app_main.h"
#include "mcal_clock.h"
#include "mcal_gpio.h"
#include "mcal_dma.h"
#include "mcal_adc.h"
#include "mcal_timer.h"
#include "mcal_spi.h"
#include "mcal_uart.h"
#include "mcal_i2c.h"
#include "bsw_bfsk.h"
#include "bsw_frame.h"
#include "app_sensor.h"
#include "app_task.h"

void app_main_init(void)
{
    /* ============ MCAL 层初始化（按依赖顺序） ============ */
    mcal_clock_init();                       /* 1. 系统时钟先起来 */
    mcal_gpio_init();                        /* 2. GPIO（无依赖） */
    mcal_uart_init(UART_ID_DEBUG, 115200);   /* 3. 调试串口先就绪，方便后续打印错误 */

    mcal_dma_init();                         /* 4. DMA（ADC 需要） */
    mcal_timer_init();                       /* 5. TIM1/2/6（ADC 触发需要 TIM6） */
    mcal_adc_init();                         /* 6. ADC1+ADC2（依赖 DMA 和 TIM6） */

    mcal_spi_init(SPI_ID_1);                 /* 7. SPI1（AD9833） */
    mcal_i2c_init(I2C_ID_1);                 /* 8. I2C1（BMP280） */

    /* ============ BSW 层初始化 ============ */
    bsw_frame_init();

    bsw_bfsk_config_t bfsk_cfg = {
        .f0 = BFSK_F0_DEFAULT,
        .f1 = BFSK_F1_DEFAULT,
        .sample_rate = BFSK_SAMPLE_RATE,
        .threshold = 1000
    };
    bsw_bfsk_init(&bfsk_cfg);

    /* ============ APP 层初始化 ============ */
    app_sensor_init();

    /* ============ FreeRTOS 任务创建 ============ */
    app_task_create_all();
    app_task_start_scheduler();
}
