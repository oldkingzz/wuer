/**
 * @file vive_navigation.h
 * @brief Vive定位导航系统 / Vive Positioning Navigation System
 * 
 * 使用双Vive传感器实现自主导航功能
 * Autonomous navigation using dual Vive sensors
 */

#ifndef VIVE_NAVIGATION_H
#define VIVE_NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 导航参数 / Navigation Parameters ========== */

/**
 * 到达目标点的距离阈值 (Vive坐标单位)
 * Distance threshold to consider target reached (in Vive coordinate units)
 */
#define NAV_ARRIVAL_THRESHOLD       100

/**
 * 朝向对齐的角度阈值 (度)
 * Heading alignment threshold in degrees
 */
#define NAV_HEADING_THRESHOLD       5.0f

/**
 * 最大线速度 (m/s)
 * Maximum linear velocity
 */
#define NAV_MAX_LINEAR_VELOCITY     0.3f

/**
 * 最大角速度 (rad/s)
 * Maximum angular velocity
 */
#define NAV_MAX_ANGULAR_VELOCITY    1.0f

/**
 * PID控制器参数 - 距离控制
 * PID parameters for distance control
 */
#define NAV_DISTANCE_KP             0.001f
#define NAV_DISTANCE_KI             0.0f
#define NAV_DISTANCE_KD             0.0f

/**
 * PID控制器参数 - 角度控制
 * PID parameters for heading control
 */
#define NAV_HEADING_KP              0.02f
#define NAV_HEADING_KI              0.0f
#define NAV_HEADING_KD              0.001f

/* ========== 数据结构 / Data Structures ========== */

/**
 * @brief Vive坐标点
 * Vive coordinate point
 */
typedef struct {
    uint16_t x;     // X坐标 (0-8191)
    uint16_t y;     // Y坐标 (0-8191)
} vive_point_t;

/**
 * @brief 机器人位姿（基于Vive坐标系）
 * Robot pose in Vive coordinate system
 */
typedef struct {
    uint16_t x;         // X位置 (Vive坐标)
    uint16_t y;         // Y位置 (Vive坐标)
    float heading;      // 朝向角度 (度, 0-360)
    bool valid;         // 位姿是否有效
} vive_pose_t;

/**
 * @brief 导航状态
 * Navigation state
 */
typedef enum {
    NAV_STATE_IDLE,         // 空闲
    NAV_STATE_NAVIGATING,   // 导航中
    NAV_STATE_ARRIVED,      // 已到达
    NAV_STATE_ERROR         // 错误
} nav_state_t;

/**
 * @brief 导航状态信息
 * Navigation status information
 */
typedef struct {
    nav_state_t state;              // 当前状态
    vive_pose_t current_pose;       // 当前位姿
    vive_point_t target;            // 目标点
    float distance_to_target;       // 到目标点的距离
    float heading_error;            // 朝向误差 (度)
    float linear_velocity;          // 当前线速度 (m/s)
    float angular_velocity;         // 当前角速度 (rad/s)
} nav_status_t;

/* ========== API函数 / API Functions ========== */

/**
 * @brief 初始化Vive导航系统
 * Initialize Vive navigation system
 * 
 * @return ESP_OK 成功 / Success
 *         ESP_FAIL 失败 / Failure
 */
esp_err_t vive_nav_init(void);

/**
 * @brief 设置目标点
 * Set target point
 * 
 * @param target_x 目标X坐标 (Vive坐标)
 * @param target_y 目标Y坐标 (Vive坐标)
 * @return ESP_OK 成功 / Success
 */
esp_err_t vive_nav_set_target(uint16_t target_x, uint16_t target_y);

/**
 * @brief 开始导航到目标点
 * Start navigation to target
 * 
 * @return ESP_OK 成功 / Success
 *         ESP_ERR_INVALID_STATE 未设置目标点 / No target set
 */
esp_err_t vive_nav_start(void);

/**
 * @brief 停止导航
 * Stop navigation
 * 
 * @return ESP_OK 成功 / Success
 */
esp_err_t vive_nav_stop(void);

/**
 * @brief 获取导航状态
 * Get navigation status
 * 
 * @param status 输出状态信息 / Output status
 * @return ESP_OK 成功 / Success
 */
esp_err_t vive_nav_get_status(nav_status_t *status);

/**
 * @brief 获取当前机器人位姿
 * Get current robot pose
 * 
 * @param pose 输出位姿 / Output pose
 * @return ESP_OK 成功 / Success
 */
esp_err_t vive_nav_get_pose(vive_pose_t *pose);

#endif // VIVE_NAVIGATION_H

