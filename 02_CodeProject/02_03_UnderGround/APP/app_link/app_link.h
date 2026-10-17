/**
 * @file    app_link.h
 * @brief   应用层链路管理 - 频点和节点关系存储
 * 
 * @details
 * 功能：
 * - 保存上下级节点的频点信息（上行频点、下行频点）
 * - 管理节点角色关系（上级/下级）
 * - 提供频点查询和更新接口
 * 
 * 架构说明：
 * - 将频点管理从 app_node_fsm 分离出来
 * - app_node_fsm 专注状态机逻辑
 * - app_link 专注频点存储和查询
 * 
 * 模块职责：
 * 1. 保存扫频后建立的频点关系（上级/下级）
 * 2. 为业务层提供频点查询接口
 * 3. 故障恢复时清除链路信息
 * 
 * 初始化：
 * - app_link_init() 在系统启动时调用
 * 
 * 使用流程：
 * 1. app_link_init() 初始化
 * 2. 收到 FREQ_PAIR 帧后，调用 app_link_set_uplink() 或 app_link_set_downlink()
 * 3. 发送业务帧前，通过 app_link_get_uplink() / app_link_get_downlink() 查询频点
 * 4. 故障或扫频失败时，调用 app_link_clear_all() 清除
 */

#ifndef APP_LINK_H
#define APP_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================ */
/* 返回值定义                                                                   */
/* ============================================================================ */

/**
 * @brief app_link 模块返回值
 * @details 所有公开 API 返回此类型
 */
typedef enum
{
    APP_LINK_OK          = 0,  /* 操作成功 */
    APP_LINK_NOT_FOUND   = 1,  /* 未找到对应的链路信息 */
    APP_LINK_ERR         = 2,  /* 其他错误（如未初始化） */
} app_link_ret_t;

/* ============================================================================ */
/* 频点对结构体                                                                  */
/* ============================================================================ */

/**
 * @brief 工作频点对（BFSK 双频通信）
 * @details 包含低频点和高频点，用于 BFSK 调制和解调
 */
typedef struct
{
    uint16_t f0_hz;  /* 低频点（Hz，取值范围 125-1000） */
    uint16_t f1_hz;  /* 高频点（Hz，取值范围 125-1000） */
} app_link_freq_pair_t;

/* ============================================================================ */
/* 初始化函数                                                                    */
/* ============================================================================ */

/**
 * @brief 初始化链路管理模块
 * 
 * @details
 * 必须在系统启动时调用一次，用于初始化内部状态。
 * 调用后模块处于就绪状态，可以接受频点设置和查询。
 * 
 * @note 非线程安全，只在初始化期间调用
 */
void app_link_init(void);

/* ============================================================================ */
/* 上行链路管理 API                                                              */
/* ============================================================================ */

/**
 * @brief 设置上级节点的频点
 * 
 * @param peer_addr    上级节点的硬件地址（0x00-0xFE）
 * @param f0_hz        低频点（Hz）
 * @param f1_hz        高频点（Hz）
 * 
 * @return app_link_ret_t
 *   - APP_LINK_OK:   成功保存频点
 *   - APP_LINK_ERR:  模块未初始化或参数无效
 * 
 * @details
 * 通常在收到来自上级的 FREQ_PAIR 帧后调用。
 * 如果已存在上级信息，将被新值覆盖。
 * 
 * @note 
 * - 参数不做有效性检查，调用方应确保频点有效
 * - 不支持多个上级节点（中继与井上的单向关系）
 * 
 * @example
 * @code
 * app_link_set_uplink(0x01, 4800, 4900);  // 保存上级 0x01 的频点
 * @endcode
 */
app_link_ret_t app_link_set_uplink(uint8_t peer_addr, uint16_t f0_hz, uint16_t f1_hz);

/**
 * @brief 获取上级节点的频点
 * 
 * @param p_freq  输出参数：频点对指针
 *                - 若为 NULL，仅检查链路有效性
 *                - 若非 NULL，返回时填充 f0_hz 和 f1_hz
 * @param p_addr  输出参数：上级节点地址指针
 *                - 若为 NULL，不输出地址
 *                - 若非 NULL，返回时填充对端地址
 * 
 * @return app_link_ret_t
 *   - APP_LINK_OK:        成功获取频点
 *   - APP_LINK_NOT_FOUND: 上级链路未建立
 *   - APP_LINK_ERR:       模块未初始化
 * 
 * @details
 * 用于业务逻辑查询上级频点，以便向上级发送数据时选择正确的调制频点。
 * 如果上级链路未建立（频点为 0），返回 APP_LINK_NOT_FOUND。
 * 
 * @example
 * @code
 * app_link_freq_pair_t freq;
 * uint8_t addr;
 * if (app_link_get_uplink(&freq, &addr) == APP_LINK_OK)
 * {
 *     // 使用 freq.f0_hz 和 freq.f1_hz 发送业务帧
 * }
 * @endcode
 */
app_link_ret_t app_link_get_uplink(app_link_freq_pair_t *p_freq, uint8_t *p_addr);

/**
 * @brief 检查上级链路是否有效
 * 
 * @return bool
 *   - true:   上级链路已建立（频点有效）
 *   - false:  上级链路未建立或模块未初始化
 * 
 * @details
 * 快速检查接口，用于判断是否可以向上级发送数据。
 * 不返回频点信息，仅判断链路状态。
 * 
 * @example
 * @code
 * if (app_link_is_uplink_valid())
 * {
 *     // 可以向上级发送数据
 * }
 * else
 * {
 *     // 上级链路未建立，需要扫频
 * }
 * @endcode
 */
bool app_link_is_uplink_valid(void);

/**
 * @brief 清除上级链路信息
 * 
 * @details
 * 用于故障恢复或重新扫频时清除已保存的上级信息。
 * 调用后上级链路恢复到未建立状态。
 * 
 * @note 非线程安全，不应在 ISR 中调用
 * 
 * @example
 * @code
 * if (link_timeout_detected())
 * {
 *     app_link_clear_uplink();  // 清除失效的链路
 *     start_scan();             // 开始重新扫频
 * }
 * @endcode
 */
void app_link_clear_uplink(void);

/* ============================================================================ */
/* 下行链路管理 API                                                              */
/* ============================================================================ */

/**
 * @brief 设置下级节点的频点
 * 
 * @param peer_addr    下级节点的硬件地址（0x00-0xFE）
 * @param f0_hz        低频点（Hz）
 * @param f1_hz        高频点（Hz）
 * 
 * @return app_link_ret_t
 *   - APP_LINK_OK:   成功保存频点
 *   - APP_LINK_ERR:  模块未初始化或参数无效
 * 
 * @details
 * 通常在收到来自下级的 FREQ_PAIR 帧后调用。
 * 如果已存在下级信息，将被新值覆盖。
 * 
 * @note 
 * - 参数不做有效性检查，调用方应确保频点有效
 * - 不支持多个下级节点（中继与井下的单向关系）
 * 
 * @example
 * @code
 * app_link_set_downlink(0x03, 4800, 4900);  // 保存下级 0x03 的频点
 * @endcode
 */
app_link_ret_t app_link_set_downlink(uint8_t peer_addr, uint16_t f0_hz, uint16_t f1_hz);

/**
 * @brief 获取下级节点的频点
 * 
 * @param p_freq  输出参数：频点对指针
 *                - 若为 NULL，仅检查链路有效性
 *                - 若非 NULL，返回时填充 f0_hz 和 f1_hz
 * @param p_addr  输出参数：下级节点地址指针
 *                - 若为 NULL，不输出地址
 *                - 若非 NULL，返回时填充对端地址
 * 
 * @return app_link_ret_t
 *   - APP_LINK_OK:        成功获取频点
 *   - APP_LINK_NOT_FOUND: 下级链路未建立
 *   - APP_LINK_ERR:       模块未初始化
 * 
 * @details
 * 用于业务逻辑查询下级频点，以便向下级发送数据时选择正确的调制频点。
 * 如果下级链路未建立（频点为 0），返回 APP_LINK_NOT_FOUND。
 * 
 * @example
 * @code
 * app_link_freq_pair_t freq;
 * uint8_t addr;
 * if (app_link_get_downlink(&freq, &addr) == APP_LINK_OK)
 * {
 *     // 使用 freq.f0_hz 和 freq.f1_hz 发送业务帧
 * }
 * @endcode
 */
app_link_ret_t app_link_get_downlink(app_link_freq_pair_t *p_freq, uint8_t *p_addr);

/**
 * @brief 检查下级链路是否有效
 * 
 * @return bool
 *   - true:   下级链路已建立（频点有效）
 *   - false:  下级链路未建立或模块未初始化
 * 
 * @details
 * 快速检查接口，用于判断是否可以向下级发送数据。
 * 不返回频点信息，仅判断链路状态。
 * 
 * @example
 * @code
 * if (app_link_is_downlink_valid())
 * {
 *     // 可以向下级发送数据
 * }
 * else
 * {
 *     // 下级链路未建立，需要扫频
 * }
 * @endcode
 */
bool app_link_is_downlink_valid(void);

/**
 * @brief 清除下级链路信息
 * 
 * @details
 * 用于故障恢复或重新扫频时清除已保存的下级信息。
 * 调用后下级链路恢复到未建立状态。
 * 
 * @note 非线程安全，不应在 ISR 中调用
 * 
 * @example
 * @code
 * if (link_timeout_detected())
 * {
 *     app_link_clear_downlink();  // 清除失效的链路
 *     start_scan();               // 开始重新扫频
 * }
 * @endcode
 */
void app_link_clear_downlink(void);

/* ============================================================================ */
/* 链路管理 API                                                                  */
/* ============================================================================ */

/**
 * @brief 清除所有链路信息（上级和下级）
 * 
 * @details
 * 用于系统故障、重启或全局重新扫频时清除所有已建立的链路。
 * 调用后上级和下级链路都恢复到未建立状态。
 * 
 * @note 非线程安全，不应在 ISR 中调用
 * 
 * @example
 * @code
 * on_system_fault():
 * {
 *     app_link_clear_all();  // 清除所有链路
 *     app_node_fsm_restart(); // 重启状态机
 * }
 * @endcode
 */
void app_link_clear_all(void);

/* ============================================================================ */
/* 总线静默检测 API                                                              */
/* ============================================================================ */

/**
 * @brief 开始监控总线静默
 * 
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 * 记录静默开始时刻，用于后续判断是否超时。
 * 通常在所有标志位为 0 时调用，开始检测链路是否完全断裂。
 * 
 * @note 非线程安全，不应在 ISR 中调用
 * 
 * @example
 * @code
 * // 检测到所有标志位为 0，开始监控总线静默
 * if (all_flags_zero())
 * {
 *     app_link_start_bus_quiet_monitor(xTaskGetTickCount());
 * }
 * @endcode
 */
void app_link_start_bus_quiet_monitor(uint32_t now_tick);

/**
 * @brief 停止监控总线静默
 * 
 * @details
 * 收到任何信号后调用，表示总线不再静默。
 * 重置静默时间计数器。
 * 
 * @note 非线程安全，不应在 ISR 中调用
 * 
 * @example
 * @code
 * // 收到信号后停止静默监控
 * on_signal_received():
 * {
 *     app_link_stop_bus_quiet_monitor();
 * }
 * @endcode
 */
void app_link_stop_bus_quiet_monitor(void);

/**
 * @brief 判断总线是否静默超时
 * 
 * @param now_tick 当前时刻（ms）
 * @return true = 总线静默超时（超过 3 秒未收到信号），false = 未超时或静默未开始
 * 
 * @details
 * 用于检测通信链路是否断裂。
 * 如果自 app_link_start_bus_quiet_monitor() 调用后超过 3 秒仍未收到信号，
 * 则认为链路完全断裂，需要重新扫频。
 * 
 * @note 非线程安全，不应在 ISR 中调用
 * 
 * @example
 * @code
 * // 周期检查总线静默超时
 * if (app_link_is_bus_quiet_timeout(xTaskGetTickCount()))
 * {
 *     // 链路断裂，进入扫频监听状态
 *     app_node_fsm_enter_scan_listen();
 * }
 * @endcode
 */
bool app_link_is_bus_quiet_timeout(uint32_t now_tick);

#ifdef __cplusplus
}
#endif

#endif /* APP_LINK_H */


