/**
 * @file motor_driver.h
 * @brief 电机驱动器接口 / Motor Driver Interface
 * 
 * 本模块提供L298N电机驱动器的控制接口，包括PWM速度控制和方向控制
 * This module provides control interface for L298N motor driver, including PWM speed control and direction control
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief 电机方向枚举
 * Motor Direction Enumeration
 */
typedef enum {
    MOTOR_STOP = 0,      /**< 电机停止 / Motor stopped */
    MOTOR_FORWARD,       /**< 电机正转 / Motor forward */
    MOTOR_BACKWARD       /**< 电机反转 / Motor backward (未使用 / Not used) */
} motor_direction_t;

/**
 * @brief 电机状态结构体
 * Motor State Structure
 */
typedef struct {
    bool is_running;              /**< 电机是否运行 / Is motor running */
    motor_direction_t direction;  /**< 电机方向 / Motor direction */
    uint32_t duty_cycle;          /**< PWM占空比 (0-1023) / PWM duty cycle (0-1023) */
    float speed_percentage;       /**< 速度百分比 (0-100%) / Speed percentage (0-100%) */
} motor_state_t;

/**
 * @brief 初始化电机驱动器
 * Initialize Motor Driver
 * 
 * 配置GPIO引脚和PWM定时器
 * Configures GPIO pins and PWM timer
 * 
 * @return 
 *     - ESP_OK: 成功 / Success
 *     - ESP_FAIL: 失败 / Failure
 */
esp_err_t motor_driver_init(void);

/**
 * @brief 设置电机方向
 * Set Motor Direction
 * 
 * 通过控制IN1和IN2引脚设置电机旋转方向
 * Sets motor rotation direction by controlling IN1 and IN2 pins
 * 
 * @param direction 电机方向 / Motor direction
 * @return 
 *     - ESP_OK: 成功 / Success
 *     - ESP_ERR_INVALID_ARG: 无效参数 / Invalid argument
 */
esp_err_t motor_set_direction(motor_direction_t direction);

/**
 * @brief 设置电机速度 (PWM占空比)
 * Set Motor Speed (PWM Duty Cycle)
 * 
 * @param duty_cycle PWM占空比值 (0-1023) / PWM duty cycle value (0-1023)
 * @return 
 *     - ESP_OK: 成功 / Success
 *     - ESP_ERR_INVALID_ARG: 占空比超出范围 / Duty cycle out of range
 */
esp_err_t motor_set_speed(uint32_t duty_cycle);

/**
 * @brief 设置电机速度 (百分比)
 * Set Motor Speed (Percentage)
 * 
 * @param percentage 速度百分比 (0.0-100.0) / Speed percentage (0.0-100.0)
 * @return 
 *     - ESP_OK: 成功 / Success
 *     - ESP_ERR_INVALID_ARG: 百分比超出范围 / Percentage out of range
 */
esp_err_t motor_set_speed_percentage(float percentage);

/**
 * @brief 启动电机
 * Start Motor
 * 
 * 使用当前设置的速度和方向启动电机
 * Starts motor with current speed and direction settings
 * 
 * @return 
 *     - ESP_OK: 成功 / Success
 */
esp_err_t motor_start(void);

/**
 * @brief 停止电机
 * Stop Motor
 * 
 * 立即停止电机运行
 * Immediately stops motor operation
 * 
 * @return 
 *     - ESP_OK: 成功 / Success
 */
esp_err_t motor_stop(void);

/**
 * @brief 获取电机当前状态
 * Get Current Motor State
 * 
 * @param state 指向状态结构体的指针 / Pointer to state structure
 * @return 
 *     - ESP_OK: 成功 / Success
 *     - ESP_ERR_INVALID_ARG: 空指针 / Null pointer
 */
esp_err_t motor_get_state(motor_state_t *state);

/**
 * @brief 电机紧急停止
 * Emergency Stop Motor
 *
 * 立即停止电机并设置速度为0
 * Immediately stops motor and sets speed to 0
 *
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t motor_emergency_stop(void);


/* ========== 电机2控制函数 / Motor 2 Control Functions ========== */

/**
 * @brief 初始化电机2驱动器
 * Initialize Motor 2 Driver
 *
 * @return
 *     - ESP_OK: 成功 / Success
 *     - ESP_FAIL: 失败 / Failure
 */
esp_err_t motor2_driver_init(void);

/**
 * @brief 设置电机2方向
 * Set Motor 2 Direction
 *
 * @param direction 电机方向 / Motor direction
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t motor2_set_direction(motor_direction_t direction);

/**
 * @brief 设置电机2速度 (PWM占空比)
 * Set Motor 2 Speed (PWM Duty Cycle)
 *
 * @param duty_cycle PWM占空比值 (0-1023) / PWM duty cycle value (0-1023)
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t motor2_set_speed(uint32_t duty_cycle);

/**
 * @brief 启动电机2
 * Start Motor 2
 *
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t motor2_start(void);

/**
 * @brief 停止电机2
 * Stop Motor 2
 *
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t motor2_stop(void);

/**
 * @brief 获取电机2当前状态
 * Get Current Motor 2 State
 *
 * @param state 指向状态结构体的指针 / Pointer to state structure
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t motor2_get_state(motor_state_t *state);

#endif // MOTOR_DRIVER_H

