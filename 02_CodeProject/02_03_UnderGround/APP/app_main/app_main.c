/* app_main.c - 井下节点 APP 层入口
 *
 * 初始化顺序：
 *   MCAL → BSW → APP → app_task_create_all()
 *
 * 注：
 *   - 当前 APP 包含 main / sensor / task 三个模块；
 *   - 所有 FreeRTOS 任务由 app_task 模块统一注册与创建；
 *   - osKernelStart() 由 main.c 调用（CubeMX 模板约定）。
 */

#include "app_main.h"
#include "main.h"

#include "mcal_clock.h"
#include "mcal_gpio.h"
#include "mcal_dma.h"
#include "mcal_adc.h"
#include "mcal_timer.h"
#include "mcal_spi.h"
#include "mcal_uart.h"
#include "mcal_i2c.h"

#include "bsw_log.h"
#include "bsw_ds18b20.h"
#include "bsw_bmp280.h"

#include "app_sensor.h"
#include "app_task.h"

/* ========== APP 层初始化入口 ========== */

void app_main_init(void)
{
    /* ============ MCAL 层初始化（按依赖顺序） ============ */
    mcal_clock_init();                       /* 1. 系统时钟先起来 */
    mcal_gpio_init();                        /* 2. GPIO（无依赖） */
    mcal_uart_init(UART_ID_DEBUG, 115200);   /* 3. 调试串口先就绪，方便后续日志 */

    mcal_dma_init();                         /* 4. DMA（ADC 需要） */
    mcal_timer_init();                       /* 5. TIM1/2/6（ADC 触发需要 TIM6） */
    mcal_adc_init();                         /* 6. ADC1+ADC2（依赖 DMA + TIM6） */

    mcal_spi_init(SPI_ID_1);                 /* 7. SPI1（AD9833） */
    mcal_i2c_init(I2C_ID_1);                 /* 8. I2C1（BMP280） */

    /* ============ BSW 层初始化 ============ */
    /* BSW 层需要时间戳 → App 层注入 HAL_GetTick 入口（CubeMX 约定） */
    bsw_ds18b20_set_tick_source(HAL_GetTick);
    bsw_bmp280_set_tick_source(HAL_GetTick);

    /* BMP280 初始化（地址 0x76，SDO 拉低；如硬件 SDO 接 VDD 改为 0x77） */
    bsw_bmp280_init(I2C_ID_1, BSW_BMP280_I2C_ADDR_0x76);

    /* ============ APP 层初始化 ============ */
    app_sensor_init();

    /* ============ BSW 日志首发（验证 MCAL UART → BSW log 链路） ============ */
    bsw_log("UnderGround node boot OK (RTOS online)\r\n");

    /* ============ 创建所有 FreeRTOS 任务（任务管理由 app_task 模块负责） ============ */
    app_task_create_all();

    /* 注意：osKernelStart() 由 main.c 调用 */
}
