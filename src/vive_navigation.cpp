/**
 * @file vive_navigation.cpp
 * @brief Vive定位导航系统实现 / Vive Navigation System Implementation
 */

#include "include/vive_navigation.h"
#include "include/vive_sensor.h"
#include "include/chassis.h"
#include "include/pid.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "VIVE_NAV";

// 导航状态变量
static nav_state_t g_nav_state = NAV_STATE_IDLE;
static vive_point_t g_target_point = {0, 0};
static vive_pose_t g_current_pose = {0, 0, 0.0f, false};
static bool g_target_set = false;
static bool g_nav_initialized = false;

// PID控制器
static pid_controller_t g_pid_distance;  // 距离控制
static pid_controller_t g_pid_heading;   // 朝向控制

// 互斥锁
static SemaphoreHandle_t g_nav_mutex = NULL;

// 导航任务句柄
static TaskHandle_t g_nav_task_handle = NULL;

// 数学常数
#define PI 3.14159265359f
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / PI)

/**
 * @brief 计算两点之间的距离
 */
static float calculate_distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    float dx = (float)((int32_t)x2 - (int32_t)x1);
    float dy = (float)((int32_t)y2 - (int32_t)y1);
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 计算从点1到点2的角度 (度)
 */
static float calculate_angle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    float dx = (float)((int32_t)x2 - (int32_t)x1);
    float dy = (float)((int32_t)y2 - (int32_t)y1);
    float angle_rad = atan2f(dy, dx);
    float angle_deg = RAD_TO_DEG(angle_rad);

    // 归一化到0-360度
    if (angle_deg < 0) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/**
 * @brief 计算角度差 (考虑360度循环)
 */
static float angle_difference(float target_angle, float current_angle)
{
    float diff = target_angle - current_angle;

    // 归一化到-180到180度
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    return diff;
}

/**
 * @brief 更新机器人位姿（基于双Vive传感器）
 */
static esp_err_t update_robot_pose(void)
{
    vive_data_t vive1, vive2;

    // 读取两个Vive传感器
    vive_read_all(&vive1, &vive2);

    // 检查数据有效性
    if (!vive1.valid || !vive2.valid) {
        g_current_pose.valid = false;
        return ESP_ERR_INVALID_STATE;
    }

    // 计算机器人中心位置（两个传感器的中点）
    g_current_pose.x = (vive1.x + vive2.x) / 2;
    g_current_pose.y = (vive1.y + vive2.y) / 2;

    // 计算机器人朝向（从传感器1指向传感器2的方向）
    g_current_pose.heading = calculate_angle(vive1.x, vive1.y, vive2.x, vive2.y);
    g_current_pose.valid = true;

    return ESP_OK;
}

/**
 * @brief 导航控制任务
 */
static void navigation_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Navigation task started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);  // 50ms控制周期 (20Hz)

    while (1) {
        if (xSemaphoreTake(g_nav_mutex, portMAX_DELAY) == pdTRUE) {

            if (g_nav_state == NAV_STATE_NAVIGATING) {
                // 更新位姿
                if (update_robot_pose() != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to update pose - Vive signal lost");
                    g_nav_state = NAV_STATE_ERROR;
                    chassis_stop();
                    xSemaphoreGive(g_nav_mutex);
                    vTaskDelayUntil(&last_wake_time, period);
                    continue;
                }

                // 计算到目标点的距离
                float distance = calculate_distance(
                    g_current_pose.x, g_current_pose.y,
                    g_target_point.x, g_target_point.y
                );

                // 检查是否到达目标
                if (distance < NAV_ARRIVAL_THRESHOLD) {
                    ESP_LOGI(TAG, "Target reached! Distance: %.1f", distance);
                    g_nav_state = NAV_STATE_ARRIVED;
                    chassis_stop();
                    xSemaphoreGive(g_nav_mutex);
                    vTaskDelayUntil(&last_wake_time, period);
                    continue;
                }

                // 计算目标角度
                float target_heading = calculate_angle(
                    g_current_pose.x, g_current_pose.y,
                    g_target_point.x, g_target_point.y
                );

                // 计算朝向误差
                float heading_error = angle_difference(target_heading, g_current_pose.heading);

                // 使用PID计算控制量
                float dt = 0.05f;  // 50ms

                // 距离PID控制线速度
                float linear_vel = pid_compute(&g_pid_distance, distance, 0.0f, dt);

                // 角度PID控制角速度
                float angular_vel = pid_compute(&g_pid_heading, 0.0f, heading_error, dt);

                // 限制速度
                if (linear_vel > NAV_MAX_LINEAR_VELOCITY) {
                    linear_vel = NAV_MAX_LINEAR_VELOCITY;
                }
                if (linear_vel < 0.0f) {
                    linear_vel = 0.0f;
                }

                if (angular_vel > NAV_MAX_ANGULAR_VELOCITY) {
                    angular_vel = NAV_MAX_ANGULAR_VELOCITY;
                }
                if (angular_vel < -NAV_MAX_ANGULAR_VELOCITY) {
                    angular_vel = -NAV_MAX_ANGULAR_VELOCITY;
                }

                // 如果朝向误差太大，先转向
                if (fabsf(heading_error) > 45.0f) {
                    linear_vel = 0.0f;  // 停止前进，只转向
                }

                // 发送速度命令到底盘
                chassis_set_velocity(linear_vel, angular_vel);

                ESP_LOGD(TAG, "Nav: Pos(%d,%d) Target(%d,%d) Dist=%.1f Head=%.1f Err=%.1f V=%.2f W=%.2f",
                         g_current_pose.x, g_current_pose.y,
                         g_target_point.x, g_target_point.y,
                         distance, g_current_pose.heading, heading_error,
                         linear_vel, angular_vel);
            }

            xSemaphoreGive(g_nav_mutex);
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief 初始化Vive导航系统
 */
esp_err_t vive_nav_init(void)
{
    if (g_nav_initialized) {
        ESP_LOGW(TAG, "Navigation already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing Vive navigation system...");

    // 创建互斥锁
    g_nav_mutex = xSemaphoreCreateMutex();
    if (g_nav_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // 初始化PID控制器
    pid_init(&g_pid_distance, NAV_DISTANCE_KP, NAV_DISTANCE_KI, NAV_DISTANCE_KD);
    pid_set_limits(&g_pid_distance, 0.0f, NAV_MAX_LINEAR_VELOCITY);

    pid_init(&g_pid_heading, NAV_HEADING_KP, NAV_HEADING_KI, NAV_HEADING_KD);
    pid_set_limits(&g_pid_heading, -NAV_MAX_ANGULAR_VELOCITY, NAV_MAX_ANGULAR_VELOCITY);

    ESP_LOGI(TAG, "PID controllers initialized");
    ESP_LOGI(TAG, "  Distance: Kp=%.4f Ki=%.4f Kd=%.4f",
             NAV_DISTANCE_KP, NAV_DISTANCE_KI, NAV_DISTANCE_KD);
    ESP_LOGI(TAG, "  Heading: Kp=%.4f Ki=%.4f Kd=%.4f",
             NAV_HEADING_KP, NAV_HEADING_KI, NAV_HEADING_KD);

    // 创建导航任务
    BaseType_t ret = xTaskCreate(
        navigation_task,
        "vive_nav",
        4096,
        NULL,
        4,  // 优先级4
        &g_nav_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create navigation task");
        vSemaphoreDelete(g_nav_mutex);
        return ESP_FAIL;
    }

    g_nav_initialized = true;
    ESP_LOGI(TAG, "Vive navigation system initialized successfully");

    return ESP_OK;
}

/**
 * @brief 设置目标点
 */
esp_err_t vive_nav_set_target(uint16_t target_x, uint16_t target_y)
{
    if (!g_nav_initialized) {
        ESP_LOGE(TAG, "Navigation not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_target_point.x = target_x;
        g_target_point.y = target_y;
        g_target_set = true;

        ESP_LOGI(TAG, "Target set to: (%d, %d)", target_x, target_y);

        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}


/**
 * @brief 开始导航到目标点
 */
esp_err_t vive_nav_start(void)
{
    if (!g_nav_initialized) {
        ESP_LOGE(TAG, "Navigation not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_target_set) {
        ESP_LOGE(TAG, "No target set");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // 重置PID控制器
        pid_reset(&g_pid_distance);
        pid_reset(&g_pid_heading);

        // 更新当前位姿
        if (update_robot_pose() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get initial pose - check Vive sensors");
            xSemaphoreGive(g_nav_mutex);
            return ESP_FAIL;
        }

        // 计算初始距离
        float initial_distance = calculate_distance(
            g_current_pose.x, g_current_pose.y,
            g_target_point.x, g_target_point.y
        );

        ESP_LOGI(TAG, "Starting navigation:");
        ESP_LOGI(TAG, "  Current: (%d, %d) Heading: %.1f°",
                 g_current_pose.x, g_current_pose.y, g_current_pose.heading);
        ESP_LOGI(TAG, "  Target: (%d, %d)", g_target_point.x, g_target_point.y);
        ESP_LOGI(TAG, "  Distance: %.1f", initial_distance);

        g_nav_state = NAV_STATE_NAVIGATING;

        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief 停止导航
 */
esp_err_t vive_nav_stop(void)
{
    if (!g_nav_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_nav_state = NAV_STATE_IDLE;
        chassis_stop();

        ESP_LOGI(TAG, "Navigation stopped");

        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief 获取导航状态
 */
esp_err_t vive_nav_get_status(nav_status_t *status)
{
    if (!g_nav_initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status->state = g_nav_state;
        status->current_pose = g_current_pose;
        status->target = g_target_point;

        if (g_current_pose.valid) {
            status->distance_to_target = calculate_distance(
                g_current_pose.x, g_current_pose.y,
                g_target_point.x, g_target_point.y
            );

            float target_heading = calculate_angle(
                g_current_pose.x, g_current_pose.y,
                g_target_point.x, g_target_point.y
            );

            status->heading_error = angle_difference(target_heading, g_current_pose.heading);
        } else {
            status->distance_to_target = 0.0f;
            status->heading_error = 0.0f;
        }

        // 获取当前底盘速度
        chassis_velocity_t vel;
        chassis_get_velocity(&vel);
        status->linear_velocity = vel.linear_velocity;
        status->angular_velocity = vel.angular_velocity;

        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief 获取当前机器人位姿
 */
esp_err_t vive_nav_get_pose(vive_pose_t *pose)
{
    if (!g_nav_initialized || pose == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *pose = g_current_pose;
        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}


