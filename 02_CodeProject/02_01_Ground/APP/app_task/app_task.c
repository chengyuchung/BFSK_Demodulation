/* app_task.c */
#include "app_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsw_bfsk.h"
#include "bsw_frame.h"
#include "bsw_scanner.h"
#include "app_fsm.h"
#include "mcal_uart.h"

/* ========== 消息队列 ========== */
static QueueHandle_t g_queue_demod;    /* 解调结果 → 业务 */
static QueueHandle_t g_queue_comm;     /* 通信帧   → 解调 */
static QueueHandle_t g_queue_sensor;   /* 传感器数据 → 业务 */

/* ========== 信号量 ========== */
static SemaphoreHandle_t g_sem_scan;   /* 扫频完成信号 */
static SemaphoreHandle_t g_sem_frame;  /* 完整帧到达信号 */

/* ========== 各任务句柄 ========== */
static TaskHandle_t g_handle_demod;
static TaskHandle_t g_handle_comm;
static TaskHandle_t g_handle_sensor;
static TaskHandle_t g_handle_app;
static TaskHandle_t g_handle_print;

/* ========== 任务函数原型 ========== */
static void task_demod(void *arg);
static void task_comm(void *arg);
static void task_sensor(void *arg);
static void task_app(void *arg);
static void task_print(void *arg);

/* ============================================================ */

void app_task_create_all(void)
{
    /* 创建消息队列 */
    g_queue_demod = xQueueCreate(16, sizeof(uint8_t));     /* 解调比特流 */
    g_queue_comm  = xQueueCreate(8,  sizeof(frame_t*));   /* 帧指针 */
    g_queue_sensor= xQueueCreate(8,  sizeof(int16_t));     /* 温度值 */

    /* 创建信号量 */
    g_sem_scan  = xSemaphoreCreateBinary();
    g_sem_frame = xSemaphoreCreateBinary();

    /* 创建任务 */
    xTaskCreate(task_demod,   TASK_NAME_DEMOD,   TASK_STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_HIGH,   &g_handle_demod);
    xTaskCreate(task_comm,    TASK_NAME_COMM,    TASK_STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_HIGH,   &g_handle_comm);
    xTaskCreate(task_sensor,  TASK_NAME_SENSOR,  TASK_STACK_SIZE_SMALL,  NULL, TASK_PRIORITY_NORMAL, &g_handle_sensor);
    xTaskCreate(task_app,     TASK_NAME_APP,     TASK_STACK_SIZE_LARGE,  NULL, TASK_PRIORITY_LOW,    &g_handle_app);
    xTaskCreate(task_print,   TASK_NAME_PRINT,   TASK_STACK_SIZE_SMALL,  NULL, TASK_PRIORITY_LOW,    &g_handle_print);
}

void app_task_delete_all(void)
{
    if (g_handle_demod)  vTaskDelete(g_handle_demod);
    if (g_handle_comm)   vTaskDelete(g_handle_comm);
    if (g_handle_sensor) vTaskDelete(g_handle_sensor);
    if (g_handle_app)    vTaskDelete(g_handle_app);
    if (g_handle_print)  vTaskDelete(g_handle_print);
}

void app_task_start_scheduler(void)
{
    vTaskStartScheduler();
    /* 不会返回 */
}

/* ============================================================ */
/* 任务实现框架（各节点按需填充具体逻辑）                      */
/* ============================================================ */

static void task_demod(void *arg)
{
    (void)arg;
    /* TODO: BFSK 解调主循环
     * 1. 等待 DMA 采样完成信号
     * 2. 调用 bsw_bfsk_demod_bit() 解调比特
     * 3. 将比特推入 g_queue_demod
     */
    for (;;) {
        /* TODO: 等待采样完成 */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void task_comm(void *arg)
{
    (void)arg;
    /* TODO: 通信任务（扫频 + 帧解析 + 帧发送）
     * 1. 调用 bsw_scanner_tick() 推进扫频状态机
     * 2. 从 g_queue_demod 接收比特流
     * 3. 调用 bsw_frame_input_byte() 逐字节解帧
     * 4. 帧完整则推入 g_queue_comm
     */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void task_sensor(void *arg)
{
    (void)arg;
    /* TODO: 传感器采集
     * 1. DS18B20 温度读取（mcal_ds18b20）
     * 2. BMP280 温压读取（mcal_i2c）
     * 3. ADC 信号强度采集
     * 4. 结果推入 g_queue_sensor
     */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void task_app(void *arg)
{
    (void)arg;
    /* TODO: 业务逻辑（各节点不同）
     * - 井下节点：数据采集 + 低功耗休眠
     * - 中继节点：透传 + 链路维护
     * - 井上节点：协议转换 + 上报
     */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void task_print(void *arg)
{
    (void)arg;
    /* TODO: 调试打印任务（低优先级）
     * 从各队列中取数据，通过 mcal_uart_printf 输出
     */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
