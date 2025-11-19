/**
 * @file chassis.h
 * @brief 差速驱动底盘控制模块 / Differential Drive Chassis Control Module
 * 
 * 实现差速驱动底盘的运动学解算和控制
 * Implements kinematics and control for differential drive chassis
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include "esp_err.h"
#include <stdbool.h>

/* ========== 底盘物理参数 / Chassis Physical Parameters ========== */

/**
 * 轮子直径 (mm)
 * Wheel diameter in millimeters
 */
#define CHASSIS_WHEEL_DIAMETER_MM   65.0f

/**
 * 轮子直径 (m)
 * Wheel diameter in meters
 */
#define CHASSIS_WHEEL_DIAMETER_M    (CHASSIS_WHEEL_DIAMETER_MM / 1000.0f)

/**
 * 两轮间距/轮距 (mm)
 * Wheel base / distance between two wheels in millimeters
 */
#define CHASSIS_WHEEL_BASE_MM       160.0f

/**
 * 两轮间距/轮距 (m)
 * Wheel base / distance between two wheels in meters
 */
#define CHASSIS_WHEEL_BASE_M        (CHASSIS_WHEEL_BASE_MM / 1000.0f)

/**
 * 轮子周长 (m)
 * Wheel circumference in meters
 */
#define CHASSIS_WHEEL_CIRCUMFERENCE (3.14159265359f * CHASSIS_WHEEL_DIAMETER_M)

/* ========== 速度限制 / Velocity Limits ========== */

/**
 * 最大线速度 (m/s)
 * Maximum linear velocity in m/s
 */
#define CHASSIS_MAX_LINEAR_VELOCITY     0.5f

/**
 * 最大角速度 (rad/s)
 * Maximum angular velocity in rad/s
 */
#define CHASSIS_MAX_ANGULAR_VELOCITY    2.0f

/**
 * 最大轮速 (RPM)
 * Maximum wheel speed in RPM
 */
#define CHASSIS_MAX_WHEEL_RPM           100

/* ========== 底盘状态结构体 / Chassis State Structure ========== */

/**
 * @brief 底盘速度状态
 * Chassis velocity state
 */
typedef struct {
    float linear_velocity;      // 线速度 (m/s) / Linear velocity
    float angular_velocity;     // 角速度 (rad/s) / Angular velocity
    float left_wheel_rpm;       // 左轮转速 (RPM) / Left wheel RPM
    float right_wheel_rpm;      // 右轮转速 (RPM) / Right wheel RPM
    bool is_moving;             // 是否运动中 / Is moving
} chassis_velocity_t;

/**
 * @brief 底盘里程计位姿
 * Chassis odometry pose
 */
typedef struct {
    float x;                    // X位置 (m) / X position
    float y;                    // Y位置 (m) / Y position
    float theta;                // 朝向角度 (rad) / Heading angle
} chassis_pose_t;

/* ========== 底盘控制函数 / Chassis Control Functions ========== */

/**
 * @brief 初始化底盘控制系统
 * Initialize chassis control system
 * 
 * 初始化电机驱动器、编码器和底盘状态
 * Initializes motor drivers, encoders and chassis state
 * 
 * @return ESP_OK 成功 / Success
 *         ESP_FAIL 失败 / Failure
 */
esp_err_t chassis_init(void);

/**
 * @brief 设置底盘速度
 * Set chassis velocity
 * 
 * 通过差速驱动逆运动学解算，将底盘线速度和角速度转换为左右轮目标RPM
 * Converts chassis linear and angular velocity to left/right wheel RPM
 * using differential drive inverse kinematics
 * 
 * @param linear_velocity 线速度 (m/s)，正值前进，负值后退 / Linear velocity, positive=forward, negative=backward
 * @param angular_velocity 角速度 (rad/s)，正值左转，负值右转 / Angular velocity, positive=left, negative=right
 * 
 * @return ESP_OK 成功 / Success
 *         ESP_ERR_INVALID_ARG 参数超出范围 / Parameters out of range
 */
esp_err_t chassis_set_velocity(float linear_velocity, float angular_velocity);

/**
 * @brief 停止底盘
 * Stop chassis
 * 
 * 立即停止所有电机
 * Immediately stops all motors
 * 
 * @return ESP_OK 成功 / Success
 */
esp_err_t chassis_stop(void);

/**
 * @brief 获取底盘当前速度状态
 * Get current chassis velocity state
 * 
 * @param velocity 输出速度状态 / Output velocity state
 * @return ESP_OK 成功 / Success
 *         ESP_ERR_INVALID_ARG 参数为空 / NULL parameter
 */
esp_err_t chassis_get_velocity(chassis_velocity_t *velocity);

/**
 * @brief 获取底盘里程计位姿
 * Get chassis odometry pose
 * 
 * 通过编码器积分计算底盘位姿
 * Calculates chassis pose by integrating encoder data
 * 
 * @param x 输出X位置 (m) / Output X position
 * @param y 输出Y位置 (m) / Output Y position
 * @param theta 输出朝向角度 (rad) / Output heading angle
 * 
 * @return ESP_OK 成功 / Success
 *         ESP_ERR_INVALID_ARG 参数为空 / NULL parameter
 */
esp_err_t chassis_get_odometry(float *x, float *y, float *theta);

/**
 * @brief 重置里程计
 * Reset odometry
 * 
 * 将里程计位姿重置为原点
 * Resets odometry pose to origin
 * 
 * @return ESP_OK 成功 / Success
 */
esp_err_t chassis_reset_odometry(void);

/**
 * @brief 更新里程计
 * Update odometry
 * 
 * 周期性调用以更新里程计位姿
 * Should be called periodically to update odometry pose
 * 
 * @param dt 时间间隔 (秒) / Time interval in seconds
 * @return ESP_OK 成功 / Success
 */
esp_err_t chassis_update_odometry(float dt);

#endif // CHASSIS_H

