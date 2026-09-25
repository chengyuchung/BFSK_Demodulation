/* app_task.c - 井下节点 */
#include "app_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsw_bfsk.h"
#include "bsw_frame.h"
#include "app_fsm.h"
#include "app_sensor.h"
#include "mcal_uart.h"

static QueueHandle_t g_queue_demod;
static QueueHandle_t g_queue_comm;
static QueueHandle_t g_queue_sensor;
static SemaphoreHandle_t g_sem_scan;
static SemaphoreHandle_t g_sem_frame;

static TaskHandle_t g_handle_demod;
static TaskHandle_t g_handle_comm;
static TaskHandle_t g_handle_sensor;
static TaskHandle_t g_handle_app;
static TaskHandle_t g_handle_print;

static void task_demod(void *arg);
static void task_comm(void *arg);
static void task_sensor(void *arg);
static void task_app(void *arg);
static void task_print(void *arg);

void app_task_create_all(void)
{
    g_queue_demod  = xQueueCreate(16, sizeof(uint8_t));
    g_queue_comm   = xQueueCreate(8,  sizeof(frame_t*));
    g_queue_sensor = xQueueCreate(8,  sizeof(int16_t));
    g_sem_scan  = xSemaphoreCreateBinary();
    g_sem_frame = xSemaphoreCreateBinary();

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
}

static void task_demod(void *arg)   { (void)arg; for (;;) { vTaskDelay(pdMS_TO_TICKS(10)); } }
static void task_comm(void *arg)    { (void)arg; for (;;) { vTaskDelay(pdMS_TO_TICKS(10)); } }
static void task_sensor(void *arg)
{
    (void)arg;
    for (;;) {
        app_sensor_task(HAL_GetTick());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
static void task_app(void *arg)     { (void)arg; for (;;) { vTaskDelay(pdMS_TO_TICKS(100)); } }
static void task_print(void *arg)   { (void)arg; for (;;) { vTaskDelay(pdMS_TO_TICKS(500)); } }
