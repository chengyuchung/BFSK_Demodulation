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
#include "bsw_relay.h"
#include "bsw_ds18b20.h"
#include "bsw_bmp280.h"
#include "bsw_ad9833.h"
#include "bsw_adc_ringbuf.h"

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

    /* 继电器初始化（控制 5V 母线）
     * 必须在所有需要 5V 供电的外设初始化之前打开！
     * active_level 依硬件驱动电路决定：NPN 低边驱动用 HIGH，光耦/PMOS 高边驱动用 LOW
     * 这里 default_state=0 先不上电，等所有外设 init 完再开 */
    bsw_relay_init(BSW_RELAY_ACTIVE_HIGH, 0);   /* 默认 OFF，安全上电 */

    /* BMP280 初始化（地址 0x76，SDO 拉低；如硬件 SDO 接 VDD 改为 0x77） */
    bsw_bmp280_init(I2C_ID_1, BSW_BMP280_I2C_ADDR_0x76);

    /* AD9833 初始化（板上 MCLK = 25 MHz，FSYNC = PC4）
     * 必须在 mcal_spi_init(SPI_ID_1) 之后调用（依赖 SPI1 句柄） */
    bsw_ad9833_init(25000000U);

    /* 5V 母线上电（此时外设全部初始化完毕） */
    bsw_relay_on();

    /* ADC1 环形缓冲区采集启动（必须在 5V 稳态下开始，避免上电尖峰污染前 40 ms 窗口）
     * 启动后 ~51 ms 环形 buffer 一圈采样满（约 4 次协议窗口叠加），
     * 之后上层方可信任 snapshot() 返回值。 */
    bsw_adc_ringbuf_init();

    /* ============ APP 层初始化 ============ */
    app_sensor_init();

    /* ============ BSW 日志首发（验证 MCAL UART → BSW log 链路） ============ */
    bsw_log("UnderGround node boot OK (RTOS online)\r\n");

    /* ============ 创建所有 FreeRTOS 任务（任务管理由 app_task 模块负责） ============ */
    app_task_create_all();

    /* 注意：osKernelStart() 由 main.c 调用 */
}
