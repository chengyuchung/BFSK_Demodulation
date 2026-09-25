/**
 * @file    app_task.c
 * @brief   APP 层 FreeRTOS 任务统一管理实现（CMSIS-RTOS v1 风格）
 *
 * CMSIS-RTOS v1 vs v2 关键差异（本项目用的是 v1）：
 *   - 句柄类型  : osThreadId        （非 osThreadId_t）
 *   - 入口函数  : void (*)(void const *)  （非 void (*)(void *)）
 *   - 创建函数  : osThreadCreate(def, arg)  （非 osThreadNew）
 *   - 定义宏    : osThreadDef(name, thread, prio, instances, stacksz)
 *                 配合 osThread(name) 取地址
 *
 * 加任务时改三处：
 *   1. 加句柄变量
 *   2. 加 entry_xxx() 任务入口
 *   3. 在 s_task_table[] 追加一行
 */

#include "app_task.h"

#include "cmsis_os.h"          /* CMSIS-RTOS v1 wrapper */
#include "main.h"              /* HAL_GetTick() */

#include "bsw_log.h"
#include "app_sensor.h"

/* ========== 任务句柄（集中管理） ========== */

static osThreadId h_task_sensor = NULL;
/* TODO: 加任务时在这里追加句柄：
 *  static osThreadId h_task_link = NULL;
 *  static osThreadId h_task_tx   = NULL;
 */

/* ========== 任务入口函数 ==========
 *
 * v1 任务签名必须是 void (*)(void const *)，不是 void (*)(void *)
 * 即使我们不用参数，也必须写 void const *。
 *
 * 任务实体保持极简：
 *   - 周期性调业务模块的 *_task(now)
 *   - osDelay() 让出 CPU
 * 复杂业务逻辑交给业务模块，任务层只做调度。
 */

static void entry_sensor(void const *arg)
{
    (void)arg;
    for (;;) {
        app_sensor_task(HAL_GetTick());
        osDelay(APP_TASK_SENSOR_TICK_MS);
    }
}

/* TODO: 加任务时在这里追加入口函数，例如：
 *
 *  static void entry_link(void const *arg)
 *  {
 *      (void)arg;
 *      for (;;) {
 *          app_link_task(HAL_GetTick());
 *          osDelay(50);
 *      }
 *  }
 */

/* ========== 任务定义（v1 宏） ==========
 *
 * osThreadDef(name, thread_func, priority, instances, stack_bytes)
 *   - 生成 const osThreadDef_t os_thread_def_<name>
 *   - 第一个名字会出现在 RTOS 调试器 / 任务名查询里
 *   - stacksize 单位是 BYTES，不是 words
 *
 * 宏会展开成 "const" 变量（非 static），所以名字必须在本编译单元内唯一。
 */

osThreadDef(Sensor, entry_sensor,
             APP_TASK_SENSOR_PRIORITY, 1, APP_TASK_SENSOR_STACK_BYTES);

/* TODO: 加任务时在这里追加，例如：
 *  osThreadDef(Link, entry_link, osPriorityNormal, 1, 512);
 */

/* ========== 任务注册表 ==========
 *
 * 每加一个任务只追加一行。
 *   name   - 日志/调试用
 *   def    - osThread(NAME) 取得宏生成的描述符地址
 *   handle - 创建后写入句柄
 */

typedef struct {
    const char          *name;
    const osThreadDef_t *def;
    osThreadId          *handle;
} task_desc_t;

static const task_desc_t s_task_table[] = {
    { "Sensor", osThread(Sensor), &h_task_sensor },

    /* TODO: 后续添加新任务直接在这里追加：
     *  { "Link",   osThread(Link),   &h_task_link   },
     *  { "Tx",     osThread(Tx),     &h_task_tx     },
     */
};

/* ========== 对外接口 ========== */

void app_task_create_all(void)
{
    const size_t n = sizeof(s_task_table) / sizeof(s_task_table[0]);

    for (size_t i = 0; i < n; ++i) {
        *s_task_table[i].handle = osThreadCreate(s_task_table[i].def, NULL);

        if (*s_task_table[i].handle == NULL) {
            bsw_log("app_task: FAILED to create task '%s'\r\n",
                    s_task_table[i].name);
        }
    }

    bsw_log("app_task: %u task(s) created\r\n", (unsigned)n);
}
