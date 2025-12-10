/**
 * @file chassis.cpp
 * @brief 差速驱动底盘控制实现 / Differential Drive Chassis Control Implementation
 */

#include <math.h>
#include <string.h>
#include "Arduino.h"      // 用于 Serial Plotter 输出 / For Serial Plotter output
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/chassis.h"
#include "include/motor_driver.h"
#include "include/encoder.h"
#include "include/gpio_config.h"
#include "include/pid.h"

static const char *TAG = "CHASSIS";

// 底盘状态变量 / Chassis state variables
static chassis_velocity_t g_chassis_velocity = {0};
static chassis_pose_t g_chassis_pose = {0};
static bool g_chassis_initialized = false;

// PID 控制器 / PID controllers
static pid_controller_t g_pid_left;   // 左轮PID / Left wheel PID
static pid_controller_t g_pid_right;  // 右轮PID / Right wheel PID

// PID控制任务句柄 / PID control task handle
static TaskHandle_t g_pid_task_handle = NULL;

// 方向切换安全保护 / Direction change safety protection
static motor_direction_t g_prev_left_direction = MOTOR_STOP;
static motor_direction_t g_prev_right_direction = MOTOR_STOP;
static bool g_direction_changing = false;

// 数学常数 / Math constants
#define PI 3.14159265359f

/**
 * @brief 限制数值在指定范围内
 * Clamp value within range
 */
static inline float clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 将轮速 (m/s) 转换为 RPM
 * Convert wheel velocity (m/s) to RPM
 */
static float velocity_to_rpm(float velocity_m_s)
{
    // RPM = (v / (π * D)) * 60
    // v: 线速度 (m/s)
    // D: 轮子直径 (m)
    return (velocity_m_s / CHASSIS_WHEEL_CIRCUMFERENCE) * 60.0f;
}

/**
 * @brief 将 RPM 转换为轮速 (m/s)
 * Convert RPM to wheel velocity (m/s)
 */
static float rpm_to_velocity(float rpm)
{
    // v = (RPM / 60) * (π * D)
    return (rpm / 60.0f) * CHASSIS_WHEEL_CIRCUMFERENCE;
}

/**
 * @brief PID速度控制任务
 * PID Speed Control Task
 *
 * 周期性读取编码器实际转速，使用PID控制器计算PWM输出
 * Periodically reads encoder RPM and uses PID to compute PWM output
 */
static void chassis_pid_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "PID control task started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);  // 50ms control period

    while (1) {
        // 计算时间间隔 / Calculate time interval
        float dt = 0.05f;  // 50ms = 0.05s

        // 读取实际转速
        // Read actual RPM
        // 注意：电机2的方向信号被反转，所以编码器读数也需要反转符号
        // Note: Motor 2 direction signal is inverted, so encoder reading needs sign inversion
        float left_actual_rpm = fabsf(-encoder2_get_rpm());   // Motor 2 = Left wheel (negate due to reversed wiring)
        float right_actual_rpm = fabsf(encoder_get_rpm());    // Motor 1 = Right wheel

	    // 获取目标转速 (可能为负值表示反向) / Get target RPM (can be negative for backward)
	    float left_target_rpm = g_chassis_velocity.left_wheel_rpm;
	    float right_target_rpm = g_chassis_velocity.right_wheel_rpm;

	    // PID 输出（用于 Serial Plotter 可视化）/ PID outputs for Serial Plotter visualization
	    float left_pwm = 0.0f;
	    float right_pwm = 0.0f;

        if (g_chassis_velocity.is_moving) {
            // 如果正在进行方向切换，等待完成
            // If direction change is in progress, wait for completion
	        if (g_direction_changing) {
	            vTaskDelayUntil(&last_wake_time, period);
	            continue;
	        }

	        // 使用PID计算PWM占空比 (使用绝对值)
	        // Use PID to compute PWM duty cycle (using absolute values)
	        float left_error = fabsf(left_target_rpm) - left_actual_rpm;
	        float right_error = fabsf(right_target_rpm) - right_actual_rpm;

	        // 死区：误差 < 3 RPM时，保持当前PWM，避免抽搐
	        // Dead zone: when error < 3 RPM, keep current PWM to avoid jitter
	        static float last_left_pwm = 0.0f;
	        static float last_right_pwm = 0.0f;

	        if (fabsf(left_error) > 3.0f) {
	            left_pwm = pid_compute(&g_pid_left, fabsf(left_target_rpm), left_actual_rpm, dt);
	            last_left_pwm = left_pwm;
	        } else {
	            left_pwm = last_left_pwm;  // 保持上次PWM
	        }

	        if (fabsf(right_error) > 3.0f) {
	            right_pwm = pid_compute(&g_pid_right, fabsf(right_target_rpm), right_actual_rpm, dt);
	            last_right_pwm = right_pwm;
	        } else {
	            right_pwm = last_right_pwm;  // 保持上次PWM
	        }

	        // 设置电机速度 / Set motor speeds
	        motor2_set_speed((uint32_t)(left_pwm + 0.5f));   // Left wheel
	        motor_set_speed((uint32_t)(right_pwm + 0.5f));   // Right wheel

	        // 调试输出：每1秒输出一次PID状态
	        static int debug_count = 0;
	        if (++debug_count >= 20) {  // 20 * 50ms = 1秒
	            ESP_LOGI(TAG, "[PID] L: target=%.1f actual=%.1f pwm=%.0f | R: target=%.1f actual=%.1f pwm=%.0f",
	                     fabsf(left_target_rpm), left_actual_rpm, left_pwm,
	                     fabsf(right_target_rpm), right_actual_rpm, right_pwm);
	            debug_count = 0;
	        }
        } else {
            // 停止时重置PID / Reset PID when stopped
            pid_reset(&g_pid_left);
            pid_reset(&g_pid_right);
            motor_set_speed(0);
            motor2_set_speed(0);
        }

	    // 等待下一个控制周期 / Wait for next control period
	    vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief 初始化底盘控制系统
 */
esp_err_t chassis_init(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "初始化差速驱动底盘 / Initializing Differential Drive Chassis");
    ESP_LOGI(TAG, "========================================");

    // 初始化底盘状态 / Initialize chassis state
    memset(&g_chassis_velocity, 0, sizeof(chassis_velocity_t));
    memset(&g_chassis_pose, 0, sizeof(chassis_pose_t));

    // 初始化PID控制器 / Initialize PID controllers
    // 平衡收敛速度和稳定性：Ki降低到2.5，Kd增大到0.2抑制震荡
    // Balance convergence speed and stability: Ki=2.5, Kd=0.2 to dampen oscillation
    pid_init(&g_pid_left, 10.0f, 2.5f, 0.2f);   // 左轮：Kp=10.0, Ki=2.5, Kd=0.2
    pid_set_limits(&g_pid_left, 0.0f, (float)MOTOR_PWM_MAX_DUTY);

    pid_init(&g_pid_right, 10.0f, 2.5f, 0.2f);  // 右轮：Kp=10.0, Ki=2.5, Kd=0.2
    pid_set_limits(&g_pid_right, 0.0f, (float)MOTOR_PWM_MAX_DUTY);

    ESP_LOGI(TAG, "PID控制器已初始化 / PID controllers initialized");
    ESP_LOGI(TAG, "  Kp=10.0, Ki=0.5, Kd=0.1");
    ESP_LOGI(TAG, "  PWM范围 / PWM Range: 0-%d", MOTOR_PWM_MAX_DUTY);
    ESP_LOGI(TAG, "");

    ESP_LOGI(TAG, "底盘物理参数 / Chassis Physical Parameters:");
    ESP_LOGI(TAG, "  轮子直径 / Wheel Diameter: %.1f mm", CHASSIS_WHEEL_DIAMETER_MM);
    ESP_LOGI(TAG, "  轮距 / Wheel Base: %.1f mm", CHASSIS_WHEEL_BASE_MM);
    ESP_LOGI(TAG, "  轮子周长 / Wheel Circumference: %.3f m", CHASSIS_WHEEL_CIRCUMFERENCE);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "速度限制 / Velocity Limits:");
    ESP_LOGI(TAG, "  最大线速度 / Max Linear Velocity: %.2f m/s", CHASSIS_MAX_LINEAR_VELOCITY);
    ESP_LOGI(TAG, "  最大角速度 / Max Angular Velocity: %.2f rad/s", CHASSIS_MAX_ANGULAR_VELOCITY);
    ESP_LOGI(TAG, "  最大轮速 / Max Wheel RPM: %d RPM", CHASSIS_MAX_WHEEL_RPM);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "电机配置 / Motor Configuration:");
    ESP_LOGI(TAG, "  右轮 / Right Wheel: 电机1 (GPIO %d/%d/%d, 编码器 GPIO %d/%d)",
             MOTOR_IN1_GPIO, MOTOR_IN2_GPIO, MOTOR_PWM_GPIO,
             ENCODER_A_GPIO, ENCODER_B_GPIO);
    ESP_LOGI(TAG, "  左轮 / Left Wheel: 电机2 (GPIO %d/%d/%d, 编码器 GPIO %d/%d)",
             MOTOR2_IN1_GPIO, MOTOR2_IN2_GPIO, MOTOR2_PWM_GPIO,
             ENCODER2_A_GPIO, ENCODER2_B_GPIO);

    // 创建PID控制任务 / Create PID control task
    BaseType_t ret = xTaskCreate(
        chassis_pid_control_task,
        "chassis_pid",
        3072,
        NULL,
        5,  // High priority for motor control
        &g_pid_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create PID control task!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PID控制任务已创建 / PID control task created");

    g_chassis_initialized = true;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "底盘初始化成功 / Chassis initialized successfully");
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}

/**
 * @brief 设置底盘速度
 */
esp_err_t chassis_set_velocity(float linear_velocity, float angular_velocity)
{
    if (!g_chassis_initialized) {
        ESP_LOGE(TAG, "底盘未初始化 / Chassis not initialized");
        return ESP_FAIL;
    }

    // 限制速度在安全范围内 / Clamp velocities to safe range
    linear_velocity = clamp(linear_velocity, -CHASSIS_MAX_LINEAR_VELOCITY, CHASSIS_MAX_LINEAR_VELOCITY);
    angular_velocity = clamp(angular_velocity, -CHASSIS_MAX_ANGULAR_VELOCITY, CHASSIS_MAX_ANGULAR_VELOCITY);

    // 差速驱动逆运动学解算 / Differential drive inverse kinematics
    // v_left = v - ω * L / 2
    // v_right = v + ω * L / 2
    // 其中: v = 线速度, ω = 角速度, L = 轮距

    float v_left_m_s = linear_velocity - (angular_velocity * CHASSIS_WHEEL_BASE_M / 2.0f);
    float v_right_m_s = linear_velocity + (angular_velocity * CHASSIS_WHEEL_BASE_M / 2.0f);

    // 转换为 RPM (保留符号表示方向) / Convert to RPM (keep sign for direction)
    float left_rpm = velocity_to_rpm(v_left_m_s);
    float right_rpm = velocity_to_rpm(v_right_m_s);

    // 限制轮速在最大RPM范围内 (双向) / Clamp wheel speeds to max RPM (bidirectional)
    left_rpm = clamp(left_rpm, -CHASSIS_MAX_WHEEL_RPM, CHASSIS_MAX_WHEEL_RPM);
    right_rpm = clamp(right_rpm, -CHASSIS_MAX_WHEEL_RPM, CHASSIS_MAX_WHEEL_RPM);

    // 确定每个轮子的方向 / Determine direction for each wheel
    motor_direction_t left_direction = MOTOR_STOP;
    motor_direction_t right_direction = MOTOR_STOP;

    if (fabsf(left_rpm) > 0.1f) {
        left_direction = (left_rpm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    }
    if (fabsf(right_rpm) > 0.1f) {
        right_direction = (right_rpm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    }

    // 更新底盘状态 / Update chassis state
    g_chassis_velocity.linear_velocity = linear_velocity;
    g_chassis_velocity.angular_velocity = angular_velocity;
    g_chassis_velocity.left_wheel_rpm = left_rpm;
    g_chassis_velocity.right_wheel_rpm = right_rpm;
    g_chassis_velocity.is_moving = (fabs(linear_velocity) > 0.001f || fabs(angular_velocity) > 0.001f);

    // 控制电机 / Control motors
    // 注意: 电机1 = 右轮, 电机2 = 左轮
    // Note: Motor 1 = Right wheel, Motor 2 = Left wheel

    if (g_chassis_velocity.is_moving) {
        // 检查是否需要改变方向 / Check if direction change is needed
        bool left_dir_change = (left_direction != MOTOR_STOP && left_direction != g_prev_left_direction && g_prev_left_direction != MOTOR_STOP);
        bool right_dir_change = (right_direction != MOTOR_STOP && right_direction != g_prev_right_direction && g_prev_right_direction != MOTOR_STOP);

        if (left_dir_change || right_dir_change) {
            ESP_LOGW(TAG, "方向切换检测 / Direction change detected!");
            ESP_LOGW(TAG, "  左轮: %d -> %d | 右轮: %d -> %d",
                     g_prev_left_direction, left_direction,
                     g_prev_right_direction, right_direction);

            // 安全停止: 先停止电机，等待，然后改变方向
            // Safe stop: Stop motors first, wait, then change direction
            g_direction_changing = true;

            motor_stop();
            motor2_stop();
            motor_set_speed(0);
            motor2_set_speed(0);
            motor_set_direction(MOTOR_STOP);
            motor2_set_direction(MOTOR_STOP);

            ESP_LOGI(TAG, "等待电机停止... / Waiting for motors to stop...");
            vTaskDelay(pdMS_TO_TICKS(100));  // 100ms delay for safety

            g_direction_changing = false;
        }

        // 启动电机 / Start motors
        motor_start();
        motor2_start();

        // 设置方向 / Set directions
        esp_err_t ret1 = motor2_set_direction(left_direction);
        esp_err_t ret2 = motor_set_direction(right_direction);

        if (ret1 != ESP_OK || ret2 != ESP_OK) {
            ESP_LOGE(TAG, "方向设置失败 / Direction setting failed");
            motor_stop();
            motor2_stop();
            return ESP_FAIL;
        }

        // 更新上一次的方向 / Update previous directions
        g_prev_left_direction = left_direction;
        g_prev_right_direction = right_direction;

        // 减少日志输出频率
        static int log_counter = 0;
        if (++log_counter >= 20) {  // 每1秒输出一次
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "底盘速度 / Chassis Velocity: v=%.3f m/s, ω=%.3f rad/s",
                     linear_velocity, angular_velocity);
            ESP_LOGI(TAG, "轮速目标 / Target Wheel RPM: 左=%.1f, 右=%.1f", left_rpm, right_rpm);
            ESP_LOGI(TAG, "轮子方向 / Wheel Directions: 左=%s, 右=%s",
                     left_direction == MOTOR_FORWARD ? "FORWARD" : (left_direction == MOTOR_BACKWARD ? "BACKWARD" : "STOP"),
                     right_direction == MOTOR_FORWARD ? "FORWARD" : (right_direction == MOTOR_BACKWARD ? "BACKWARD" : "STOP"));
            ESP_LOGI(TAG, "编码器原始读数 / Raw Encoder: 左=%.1f, 右=%.1f",
                     encoder2_get_rpm(), encoder_get_rpm());
            ESP_LOGI(TAG, "========================================");
            log_counter = 0;
        }
    } else {
        // 停止电机 / Stop motors
        motor_stop();
        motor2_stop();
        motor_set_speed(0);
        motor2_set_speed(0);
        motor_set_direction(MOTOR_STOP);
        motor2_set_direction(MOTOR_STOP);

        // 重置方向记录 / Reset direction tracking
        g_prev_left_direction = MOTOR_STOP;
        g_prev_right_direction = MOTOR_STOP;

        // 强制清零底盘速度状态，防止残留
        g_chassis_velocity.left_wheel_rpm = 0.0f;
        g_chassis_velocity.right_wheel_rpm = 0.0f;
        g_chassis_velocity.linear_velocity = 0.0f;
        g_chassis_velocity.angular_velocity = 0.0f;
    }

    return ESP_OK;
}

/**
 * @brief 停止底盘
 */
esp_err_t chassis_stop(void)
{
    if (!g_chassis_initialized) {
        return ESP_FAIL;
    }
    
    // 停止所有电机 / Stop all motors
    motor_stop();
    motor2_stop();
    motor_set_speed(0);
    motor2_set_speed(0);
    
    // 重置速度状态 / Reset velocity state
    memset(&g_chassis_velocity, 0, sizeof(chassis_velocity_t));
    
    ESP_LOGI(TAG, "底盘已停止 / Chassis stopped");
    
    return ESP_OK;
}

/**
 * @brief 获取底盘当前速度状态
 */
esp_err_t chassis_get_velocity(chassis_velocity_t *velocity)
{
    if (velocity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(velocity, &g_chassis_velocity, sizeof(chassis_velocity_t));
    return ESP_OK;
}

/**
 * @brief 获取底盘里程计位姿
 */
esp_err_t chassis_get_odometry(float *x, float *y, float *theta)
{
    if (x == NULL || y == NULL || theta == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *x = g_chassis_pose.x;
    *y = g_chassis_pose.y;
    *theta = g_chassis_pose.theta;
    
    return ESP_OK;
}

/**
 * @brief 重置里程计
 */
esp_err_t chassis_reset_odometry(void)
{
    memset(&g_chassis_pose, 0, sizeof(chassis_pose_t));
    ESP_LOGI(TAG, "里程计已重置 / Odometry reset");
    return ESP_OK;
}

/**
 * @brief 更新里程计
 */
esp_err_t chassis_update_odometry(float dt)
{
    if (!g_chassis_initialized || dt <= 0.0f) {
        return ESP_FAIL;
    }
    
    // 读取编码器实际转速 / Read actual encoder RPM
    float left_actual_rpm = encoder2_get_rpm();
    float right_actual_rpm = encoder_get_rpm();
    
    // 转换为轮速 (m/s) / Convert to wheel velocity
    float v_left = rpm_to_velocity(left_actual_rpm);
    float v_right = rpm_to_velocity(right_actual_rpm);
    
    // 差速驱动正运动学 / Differential drive forward kinematics
    // v = (v_left + v_right) / 2
    // ω = (v_right - v_left) / L
    float v = (v_left + v_right) / 2.0f;
    float omega = (v_right - v_left) / CHASSIS_WHEEL_BASE_M;
    
    // 更新位姿 / Update pose
    g_chassis_pose.theta += omega * dt;
    g_chassis_pose.x += v * cosf(g_chassis_pose.theta) * dt;
    g_chassis_pose.y += v * sinf(g_chassis_pose.theta) * dt;
    
    return ESP_OK;
}

