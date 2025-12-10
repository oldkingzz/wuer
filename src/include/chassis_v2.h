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

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_V2_H

