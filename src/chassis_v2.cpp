/**
 * @file chassis_v2.cpp
 * @brief 底盘控制系统 V2 - 完全重写
 * Chassis Control System V2 - Complete rewrite
 *
 * 架构：
 * 1. 底盘层：接收(linear_vel, angular_vel)，输出电机控制
 * 2. PID层：纯算法，计算控制量
 * 3. 硬件层：电机驱动 + 编码器
 *
 * 功能：
 * - 差速运动学：(v, ω) → (left_rpm, right_rpm)
 * - PID速度控制：(target_rpm, actual_rpm) → pwm
 * - 电机控制：设置方向 + PWM
 */

#include "include/chassis_v2.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/encoder.h"
#include "include/motor_driver.h"
#include "include/pid_v2.h"
#include <Arduino.h>
#include <math.h>

static const char *TAG = "CHASSIS_V2";

// ========== 配置参数 ==========

// 硬件配置：电机和编码器方向
// 如果你交换了IN1和IN2，设置MOTOR_REVERSED=true
// 如果编码器读数与期望方向相反，设置ENCODER_REVERSED=true
#define MOTOR1_REVERSED true    // 右轮电机：IN1和IN2已交换
#define MOTOR2_REVERSED true    // 左轮电机：IN1和IN2已交换
#define ENCODER1_REVERSED false // 右轮编码器
#define ENCODER2_REVERSED                                                      \
  true // 左轮编码器（修正：物理读数为负，软件取反以修正里程计）

// 底盘物理参数
#define WHEEL_BASE_M 0.15f      // 轮距 (m)
#define WHEEL_DIAMETER_M 0.065f // 轮径 (m)
#define MAX_LINEAR_VEL 0.5f     // 最大线速度 (m/s)
#define MAX_ANGULAR_VEL 2.0f    // 最大角速度 (rad/s)
#define MAX_WHEEL_RPM 150.0f    // 最大轮速 (RPM)

// PID参数 - 左轮（正常）
#define PID_LEFT_KP 10.0f
#define PID_LEFT_KI 3.0f
#define PID_LEFT_KD 0.2f

// PID参数 - 右轮（电机漏液，需要特殊调整）
#define PID_RIGHT_KP 10.0f // 保持不变
#define PID_RIGHT_KI 1.5f  // 降低Ki，减少震荡（从3.0降到1.5）
#define PID_RIGHT_KD 0.5f  // 增大Kd，增强阻尼（从0.2增到0.5）

// PID输出范围
#define PID_OUTPUT_MIN 0.0f
#define PID_OUTPUT_MAX 1023.0f

// 控制周期
#define CONTROL_PERIOD_MS 50 // 50ms = 20Hz

// ========== 全局变量 ==========

// PID控制器
static pid_v2_t g_pid_left;
static pid_v2_t g_pid_right;

// 目标速度
static struct {
  float linear_vel;  // 线速度 (m/s)
  float angular_vel; // 角速度 (rad/s)
  float left_rpm;    // 左轮目标RPM
  float right_rpm;   // 右轮目标RPM
  bool is_moving;    // 是否运动中
} g_target;

// 控制任务句柄
static TaskHandle_t g_control_task = NULL;

// Odometry state
static struct {
  float x;             // Position X (m)
  float y;             // Position Y (m)
  float theta;         // Heading (rad, -PI ~ PI)
  float left_total_m;  // Left wheel total distance (m)
  float right_total_m; // Right wheel total distance (m)
} g_odom = {0};

// ========== 工具函数 ==========

static inline float clamp_f(float val, float min, float max) {
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

// 速度转RPM：v (m/s) → RPM
static inline float velocity_to_rpm(float vel_m_s) {
  return (vel_m_s / (M_PI * WHEEL_DIAMETER_M)) * 60.0f;
}

// ========== 硬件抽象层 ==========

// 电机控制（自动处理方向反转）
static void set_motor_right_forward() {
  motor_set_direction(MOTOR1_REVERSED ? MOTOR_BACKWARD : MOTOR_FORWARD);
}

static void set_motor_right_backward() {
  motor_set_direction(MOTOR1_REVERSED ? MOTOR_FORWARD : MOTOR_BACKWARD);
}

static void set_motor_left_forward() {
  motor2_set_direction(MOTOR2_REVERSED ? MOTOR_BACKWARD : MOTOR_FORWARD);
}

static void set_motor_left_backward() {
  motor2_set_direction(MOTOR2_REVERSED ? MOTOR_FORWARD : MOTOR_BACKWARD);
}

// 编码器读取（自动处理方向反转）
static float get_left_rpm() {
  float rpm = encoder2_get_rpm();
  return ENCODER2_REVERSED ? -rpm : rpm;
}

static float get_right_rpm() {
  float rpm = encoder_get_rpm();
  return ENCODER1_REVERSED ? -rpm : rpm;
}

// ========== 控制任务 ==========

static void control_task(void *param) {
  ESP_LOGI(TAG, "Control task started (20Hz)");

  TickType_t last_wake = xTaskGetTickCount();
  const float dt = CONTROL_PERIOD_MS / 1000.0f; // 0.05秒

  while (1) {
    if (g_target.is_moving) {
      // 1. 读取编码器（实际转速）
      float left_actual = fabsf(get_left_rpm());
      float right_actual = fabsf(get_right_rpm());

      // 2. 获取目标转速
      float left_target = fabsf(g_target.left_rpm);
      float right_target = fabsf(g_target.right_rpm);

      // 3. PID计算PWM
      float left_pwm =
          pid_v2_compute(&g_pid_left, left_target, left_actual, dt);
      float right_pwm =
          pid_v2_compute(&g_pid_right, right_target, right_actual, dt);

      // 4. 设置电机方向
      if (g_target.left_rpm > 0.1f) {
        set_motor_left_forward();
      } else if (g_target.left_rpm < -0.1f) {
        set_motor_left_backward();
      }

      if (g_target.right_rpm > 0.1f) {
        set_motor_right_forward();
      } else if (g_target.right_rpm < -0.1f) {
        set_motor_right_backward();
      }

      // 5. 设置PWM
      motor2_set_speed((uint32_t)(left_pwm + 0.5f)); // 左轮
      motor_set_speed((uint32_t)(right_pwm + 0.5f)); // 右轮

      // 6. 调试输出（每1秒）
      static int debug_count = 0;
      if (++debug_count >= 20) {
        ESP_LOGI(TAG,
                 "[LEFT] target=%.1f actual=%.1f pwm=%.0f | [RIGHT] "
                 "target=%.1f actual=%.1f pwm=%.0f",
                 left_target, left_actual, left_pwm, right_target, right_actual,
                 right_pwm);
        debug_count = 0;
      }
    } else {
      // 停止所有电机
      motor_stop();
      motor2_stop();
      motor_set_speed(0);
      motor2_set_speed(0);

      // 重置PID
      pid_v2_reset(&g_pid_left);
      pid_v2_reset(&g_pid_right);
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

// ========== 公共API ==========

esp_err_t chassis_v2_init(void) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Initializing Chassis V2...");
  ESP_LOGI(TAG, "========================================");

  // 1. 初始化PID控制器（左右轮分别调整）
  pid_v2_init(&g_pid_left, PID_LEFT_KP, PID_LEFT_KI, PID_LEFT_KD,
              PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  pid_v2_init(&g_pid_right, PID_RIGHT_KP, PID_RIGHT_KI, PID_RIGHT_KD,
              PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  ESP_LOGI(TAG, "PID initialized:");
  ESP_LOGI(TAG, "  Left:  Kp=%.1f Ki=%.1f Kd=%.1f", PID_LEFT_KP, PID_LEFT_KI,
           PID_LEFT_KD);
  ESP_LOGI(TAG, "  Right: Kp=%.1f Ki=%.1f Kd=%.1f (adjusted for motor issue)",
           PID_RIGHT_KP, PID_RIGHT_KI, PID_RIGHT_KD);

  // 2. 初始化目标状态
  g_target.linear_vel = 0.0f;
  g_target.angular_vel = 0.0f;
  g_target.left_rpm = 0.0f;
  g_target.right_rpm = 0.0f;
  g_target.is_moving = false;

  // 3. 创建控制任务
  BaseType_t ret =
      xTaskCreatePinnedToCore(control_task, "chassis_v2", 3072, NULL,
                              5, // 高优先级
                              &g_control_task,
                              1 // Core 1（实时控制）
      );

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create control task!");
    return ESP_FAIL;
  }

  // 4. 打印配置信息
  ESP_LOGI(TAG, "Hardware Configuration:");
  ESP_LOGI(TAG, "  Motor1 (Right) reversed: %s",
           MOTOR1_REVERSED ? "YES" : "NO");
  ESP_LOGI(TAG, "  Motor2 (Left) reversed: %s", MOTOR2_REVERSED ? "YES" : "NO");
  ESP_LOGI(TAG, "  Encoder1 (Right) reversed: %s",
           ENCODER1_REVERSED ? "YES" : "NO");
  ESP_LOGI(TAG, "  Encoder2 (Left) reversed: %s",
           ENCODER2_REVERSED ? "YES" : "NO");
  ESP_LOGI(TAG, "Chassis Parameters:");
  ESP_LOGI(TAG, "  Wheel base: %.3f m", WHEEL_BASE_M);
  ESP_LOGI(TAG, "  Wheel diameter: %.3f m", WHEEL_DIAMETER_M);
  ESP_LOGI(TAG, "  Max linear vel: %.2f m/s", MAX_LINEAR_VEL);
  ESP_LOGI(TAG, "  Max angular vel: %.2f rad/s", MAX_ANGULAR_VEL);
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Chassis V2 initialized successfully!");
  ESP_LOGI(TAG, "========================================");

  return ESP_OK;
}

esp_err_t chassis_v2_set_velocity(float linear, float angular) {
  // Debug Log (Rate Limited) to find out who is controlling motors
  static TickType_t last_log = 0;
  TickType_t now = xTaskGetTickCount();
  if ((now - last_log) >= pdMS_TO_TICKS(200)) { // Print 5 times a second
    Serial.printf("CHASSIS_V2: CMD lin=%.2f ang=%.2f\n", linear, angular);
    last_log = now;
  }

  // Fix: Reverse linear direction (Joystick Forward was Robot Backward).
  // This ensures Positive Linear = Robot Moves Forward = Joystick Forward.
  linear = -linear;
  // Fix: Reverse angular direction. PID Negative (Right) caused Robot Left
  // Turn.
  angular = -angular;

  // 1. 限幅输入
  linear = clamp_f(linear, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  angular = clamp_f(angular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

  // 2. 差速运动学：(v, ω) → (v_left, v_right)
  //    v_left = v - ω * L / 2
  //    v_right = v + ω * L / 2
  float v_left = linear - (angular * WHEEL_BASE_M / 2.0f);
  float v_right = linear + (angular * WHEEL_BASE_M / 2.0f);

  // 3. 速度转RPM
  float left_rpm = velocity_to_rpm(v_left);
  float right_rpm = velocity_to_rpm(v_right);

  // 4. 限幅RPM
  left_rpm = clamp_f(left_rpm, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
  right_rpm = clamp_f(right_rpm, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);

  // 5. 更新目标
  g_target.linear_vel = linear;
  g_target.angular_vel = angular;
  g_target.left_rpm = left_rpm;
  g_target.right_rpm = right_rpm;
  g_target.is_moving = (fabsf(linear) > 0.001f || fabsf(angular) > 0.001f);

  // 6. 调试输出（节流）
  static TickType_t last_log_info = 0;
  TickType_t now_info = xTaskGetTickCount();
  if ((now_info - last_log_info) >= pdMS_TO_TICKS(1000)) {
    ESP_LOGI(TAG, "Set velocity: v=%.3f ω=%.3f → L=%.1f R=%.1f RPM", linear,
             angular, left_rpm, right_rpm);
    last_log_info = now_info;
  }

  return ESP_OK;
}

esp_err_t chassis_v2_stop(void) { return chassis_v2_set_velocity(0.0f, 0.0f); }

// ========== Odometry Functions ==========

esp_err_t chassis_v2_reset_odometry(void) {
  g_odom.x = 0.0f;
  g_odom.y = 0.0f;
  g_odom.theta = 0.0f;
  g_odom.left_total_m = 0.0f;
  g_odom.right_total_m = 0.0f;
  ESP_LOGI(TAG, "Odometry reset to (0, 0, 0)");
  return ESP_OK;
}

esp_err_t chassis_v2_update_odometry(float dt) {
  // Read current RPM from encoders
  float left_rpm = get_left_rpm();
  float right_rpm = get_right_rpm();

  // Convert RPM to linear velocity (m/s)
  // v = RPM * π * diameter / 60
  const float rpm_to_mps = (M_PI * WHEEL_DIAMETER_M) / 60.0f;
  float v_left = left_rpm * rpm_to_mps;
  float v_right = right_rpm * rpm_to_mps;

  // Accumulate wheel distances
  g_odom.left_total_m += v_left * dt;
  g_odom.right_total_m += v_right * dt;

  // Differential drive kinematics
  float v_linear = (v_left + v_right) / 2.0f;
  float v_angular = (v_right - v_left) / WHEEL_BASE_M;

  // Update pose
  g_odom.x += v_linear * cosf(g_odom.theta) * dt;
  g_odom.y += v_linear * sinf(g_odom.theta) * dt;
  g_odom.theta += v_angular * dt;

  // Normalize theta to [-PI, PI]
  while (g_odom.theta > M_PI)
    g_odom.theta -= 2.0f * M_PI;
  while (g_odom.theta < -M_PI)
    g_odom.theta += 2.0f * M_PI;

  return ESP_OK;
}

esp_err_t chassis_v2_get_wheel_dist_m(float *left_total, float *right_total) {
  if (left_total == NULL || right_total == NULL) {
    ESP_LOGE(TAG, "get_wheel_dist_m: NULL pointer");
    return ESP_FAIL;
  }

  *left_total = g_odom.left_total_m;
  *right_total = g_odom.right_total_m;
  return ESP_OK;
}

esp_err_t chassis_v2_get_odometry(float *x, float *y, float *theta) {
  if (x == NULL || y == NULL || theta == NULL) {
    ESP_LOGE(TAG, "get_odometry: NULL pointer");
    return ESP_FAIL;
  }

  *x = g_odom.x;
  *y = g_odom.y;
  *theta = g_odom.theta;
  return ESP_OK;
}
