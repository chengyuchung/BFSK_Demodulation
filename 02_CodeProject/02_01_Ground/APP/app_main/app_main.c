/* app_main.c - 井上节点 */
#include "app_main.h"
#include "app_task.h"
#include "mcal_gpio.h"
#include "mcal_uart.h"
#include "mcal_adc.h"
#include "mcal_timer.h"
#include "bsw_bfsk.h"
#include "bsw_scanner.h"
#include "bsw_crc.h"
#include "bsw_frame.h"

void app_main_init(void)
{
    /* 外设初始化 */
    mcal_gpio_init();
    mcal_uart_init(UART_ID_DEBUG, 115200);
    mcal_timer_init();
    mcal_adc_init();

    /* BSW 层初始化 */
    bsw_crc_init();
    bsw_frame_init();

    bsw_bfsk_config_t bfsk_cfg = {
        .f0          = BFSK_F0_DEFAULT,
        .f1          = BFSK_F1_DEFAULT,
        .sample_rate = BFSK_SAMPLE_RATE,
        .threshold   = 1000
    };
    bsw_bfsk_init(&bfsk_cfg);

    /* 创建 FreeRTOS 任务 */
    app_task_create_all();

    /* 启动调度器，永不返回 */
    app_task_start_scheduler();
}
