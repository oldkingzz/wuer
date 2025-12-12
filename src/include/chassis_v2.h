/**
 * @file chassis_v2.h
 * @brief 底盘控制系统 V2 - 头文件
 * Chassis Control System V2 - Header
 */

#ifndef CHASSIS_V2_H
#define CHASSIS_V2_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ========== Constants ==========

#define CHASSIS_V2_WHEEL_DIAMETER_M 0.065f
#define CHASSIS_V2_WHEEL_BASE_M 0.15f
#define CHASSIS_V2_MAX_LINEAR_VEL 0.5f  // m/s
#define CHASSIS_V2_MAX_ANGULAR_VEL 2.0f // rad/s

// ========== Structures ==========

typedef struct {
  float x;     // Meters
  float y;     // Meters
  float theta; // Radians (-PI ~ PI)
} chassis_v2_pose_t;

// ========== API ==========

/**
 * @brief 初始化底盘控制系统
 * Initialize chassis control system
 *
 * @return ESP_OK on success
 */
esp_err_t chassis_v2_init(void);

/**
 * @brief 设置底盘速度
 * Set chassis velocity
 *
 * @param linear 线速度 (m/s), 范围 [-0.5, 0.5]
 * @param angular 角速度 (rad/s), 范围 [-2.0, 2.0]
 * @return ESP_OK on success
 */
esp_err_t chassis_v2_set_velocity(float linear, float angular);

/**
 * @brief 停止底盘
 * Stop chassis
 *
 * @return ESP_OK on success
 */
esp_err_t chassis_v2_stop(void);

/**
 * @brief 更新底盘里程计 (Dead Reckoning)
 * Update chassis odometry based on encoder readings.
 * Should be called periodically (e.g. from nav task).
 *
 * @param dt Time step in seconds
 */
esp_err_t chassis_v2_update_odometry(float dt);

/**
 * @brief 获取当前里程计位姿
 * Get current estimated pose from encoders.
 */
esp_err_t chassis_v2_get_odometry(float *x, float *y, float *theta);

/**
 * @brief 重置里程计为 (0,0,0)
 */
esp_err_t chassis_v2_reset_odometry(void);

/**
 * @brief Get total accumulated wheel distance (meters)
 * Useful for EKF prediction inputs.
 */
esp_err_t chassis_v2_get_wheel_dist_m(float *left_total, float *right_total);

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_V2_H
