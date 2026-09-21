/* app_main.c - 井下节点 */
#include "app_main.h"
#include "app_task.h"
#include "mcal_gpio.h"
#include "mcal_uart.h"
#include "mcal_adc.h"
#include "mcal_timer.h"
#include "mcal_ds18b20.h"
#include "bsw_bfsk.h"
#include "bsw_scanner.h"
#include "bsw_crc.h"
#include "bsw_frame.h"

void app_main_init(void)
{
    mcal_gpio_init();
    mcal_uart_init(UART_ID_DEBUG, 115200);
    mcal_timer_init();
    mcal_adc_init();
    bsw_crc_init();
    bsw_frame_init();

    /* 井下节点特有：DS18B20 初始化 */
    mcal_ds18b20_init();

    bsw_bfsk_config_t bfsk_cfg = {
        .f0 = BFSK_F0_DEFAULT, .f1 = BFSK_F1_DEFAULT,
        .sample_rate = BFSK_SAMPLE_RATE, .threshold = 1000
    };
    bsw_bfsk_init(&bfsk_cfg);
    app_task_create_all();
    app_task_start_scheduler();
}
