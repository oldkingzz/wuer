/**
 * @file user_input.h
 * @brief 用户输入接口 / User Input Interface
 *
 * 此模块已废弃，所有用户输入功能已由Web界面替代
 * This module is deprecated, all user input functions replaced by web interface
 */

#ifndef USER_INPUT_H
#define USER_INPUT_H

#include "esp_err.h"
#include <stdint.h>

/* ========== Web服务器接口 / Web Server Interface ========== */

/**
 * @brief 初始化Web服务器
 * Initialize Web Server
 *
 * 初始化WiFi和HTTP服务器，使用静态IP配置
 * Initializes WiFi and HTTP server with static IP configuration
 *
 * @return
 *     - ESP_OK: 成功 / Success
 *     - ESP_FAIL: 失败 / Failure
 */
esp_err_t web_server_init(void);

/**
 * @brief 获取底盘线速度
 * Get Chassis Linear Velocity
 *
 * 返回Web界面摇杆设置的底盘线速度 (m/s)
 * Returns chassis linear velocity set by web joystick (m/s)
 *
 * @return 线速度 (m/s) / Linear velocity
 */
float web_server_get_linear_velocity(void);

/**
 * @brief 获取底盘角速度
 * Get Chassis Angular Velocity
 *
 * 返回Web界面摇杆设置的底盘角速度 (rad/s)
 * Returns chassis angular velocity set by web joystick (rad/s)
 *
 * @return 角速度 (rad/s) / Angular velocity
 */
float web_server_get_angular_velocity(void);

/**
 * @brief 检查手动控制是否启用
 * Check if manual control is enabled
 *
 * 当处于自动模式（寻墙、导航）时，手动控制被禁用
 * Manual control is disabled when in autonomous mode (wall following,
 * navigation)
 *
 * @return true 手动控制启用 / Manual control enabled
 *         false 手动控制禁用 / Manual control disabled
 */
bool web_server_is_manual_control_enabled(void);

/**
 * @brief Get and reset the wifi packet counter
 * @return Number of packets received since last call
 */
uint32_t web_server_get_packet_count_reset(void);

#endif // USER_INPUT_H
