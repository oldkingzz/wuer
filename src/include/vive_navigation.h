/**
 * @file vive_navigation.h
 * @brief A* Path Planning Navigation System with Vive Localization
 *
 * A*路径规划导航系统 - 使用Vive定位和栅格地图
 * A* path planning navigation system using Vive localization and grid map
 */

#ifndef VIVE_NAVIGATION_H
#define VIVE_NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "astar.h"
#include "grid_map.h"
#include "robot_config.h"   //  ROBOT_BASE_LINEAR_SPEED

/* ========== 导航参数 / Navigation Parameters ========== */

/**
 * 到达目标点的距离阈值 (像素单位)
 * Distance threshold to consider target reached (in pixels)
 */
#define NAV_ARRIVAL_THRESHOLD       10  // 10 pixels = 5 inches

/**
 * 朝向对齐的角度阈值 (度)
 * Heading alignment threshold in degrees
 */
#define NAV_HEADING_THRESHOLD       5.0f

/**
 * 最大线速度 (m/s)
 * Maximum linear velocity
 *
 * 这里与 ROBOT_BASE_LINEAR_SPEED 保持一致，这样任
 * 手动/
 */
#define NAV_MAX_LINEAR_VELOCITY     ROBOT_BASE_LINEAR_SPEED

/**
 * 最大角速度 (rad/s)
 * Maximum angular velocity
 */
#define NAV_MAX_ANGULAR_VELOCITY    1.0f

/**
 * 路径跟踪前瞻距离 (像素)
 * Path tracking lookahead distance (pixels)
 */
#define NAV_LOOKAHEAD_DISTANCE      20  // 20 pixels = 10 inches

/**
 * 路径重规划距离阈值 (像素)
 * Distance threshold for path replanning (pixels)
 */
#define NAV_REPLAN_THRESHOLD        50  // 50 pixels = 25 inches

/**
 * 比例控制增益 - 距离控制
 * Proportional gain for distance control
 * 速度 = 距离 * Kp (像素 -> m/s)
 */
#define NAV_DISTANCE_KP             0.01f

/**
 * 比例控制增益 - 角度控制
 * Proportional gain for heading control
 * 角速度 = 角度误差 * Kp (度 -> rad/s)
 */
#define NAV_HEADING_KP              0.02f

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
    NAV_STATE_PLANNING,     // 路径规划中
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
    vive_pose_t current_pose;       // 当前位姿 (Vive坐标)
    vive_point_t target;            // 目标点 (Vive坐标)
    map_point_t current_map_pos;    // 当前地图位置 (像素)
    map_point_t target_map_pos;     // 目标地图位置 (像素)
    uint16_t path_length;           // 路径长度
    uint16_t current_waypoint;      // 当前路径点索引
    float distance_to_target;       // 到目标点的距离 (像素)
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
 * @brief 设置目标点并规划路径
 * Set target point and plan path
 *
 * @param target_x 目标X坐标 (Vive坐标 0-8191)
 * @param target_y 目标Y坐标 (Vive坐标 0-8191)
 * @return ESP_OK 成功 / Success
 *         ESP_FAIL 路径规划失败 / Path planning failed
 */
esp_err_t vive_nav_set_target(uint16_t target_x, uint16_t target_y);

/**
 * @brief 设置目标点（地图坐标）并规划路径
 * Set target point (map coordinates) and plan path
 *
 * @param map_x 目标X坐标 (像素 0-287)
 * @param map_y 目标Y坐标 (像素 0-119)
 * @return ESP_OK 成功 / Success
 *         ESP_FAIL 路径规划失败 / Path planning failed
 */
esp_err_t vive_nav_set_target_map(int16_t map_x, int16_t map_y);

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

/**
 * @brief 获取当前路径
 * Get current path
 *
 * @param path 输出路径 / Output path
 * @return ESP_OK 成功 / Success
 */
esp_err_t vive_nav_get_path(path_t *path);

/**
 * @brief 强制重新规划路径
 * Force path replanning
 *
 * @return ESP_OK 成功 / Success
 */
esp_err_t vive_nav_replan(void);

#endif // VIVE_NAVIGATION_H

