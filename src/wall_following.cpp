/**
 * @file wall_following.cpp
 * @brief Wall Following Algorithm Implementation
 */

#include "include/wall_following.h"
#include "include/chassis.h"
#include "include/tof_sensor.h"
#include "include/imu_sensor.h"
#include "include/encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "WALL_FOLLOW";

/* ========== 全局变量 / Global Variables ========== */

static wall_follow_state_t g_current_state = WF_STATE_IDLE;
static float g_target_heading = 0.0f;
static float g_initial_heading = 0.0f;
static float g_clearance_start_distance = 0.0f;

// PID控制器变量
static float g_pid_integral = 0.0f;
static float g_pid_prev_error = 0.0f;

// 传感器数据
static uint16_t g_tof_front = 0;
static uint16_t g_tof_left = 0;
static uint16_t g_tof_right = 0;
static float g_current_heading = 0.0f;
static float g_heading_offset = 0.0f;
static float g_total_distance = 0.0f;

// 任务句柄和互斥锁
static TaskHandle_t g_wall_follow_task_handle = NULL;
static TaskHandle_t g_sensor_update_task_handle = NULL;
static SemaphoreHandle_t g_state_mutex = NULL;
static bool g_initialized = false;
static bool g_is_running = false;

/* ========== 辅助函数 / Helper Functions ========== */

/**
 * @brief 角度归一化到 [-180, 180]
 */
static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 计算两个角度之间的最小差值
 */
static float angle_difference(float target, float current)
{
    float diff = target - current;
    return normalize_angle(diff);
}

/**
 * @brief PID控制器（循墙距离控制）
 */
static float wall_follow_pid(float target_distance, float current_distance, float dt)
{
    float error = target_distance - current_distance;

    g_pid_integral += error * dt;
    float derivative = (error - g_pid_prev_error) / dt;
    g_pid_prev_error = error;

    float output = WF_PID_KP * error +
                   WF_PID_KI * g_pid_integral +
                   WF_PID_KD * derivative;

    return output;
}

/**
 * @brief 重置PID控制器
 */
static void reset_pid(void)
{
    g_pid_integral = 0.0f;
    g_pid_prev_error = 0.0f;
}

/* ========== 传感器更新任务 / Sensor Update Task ========== */

/**
 * @brief 传感器数据更新任务
 */
static void sensor_update_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50); // 20Hz

    ESP_LOGI(TAG, "Sensor update task started");

    while (1) {
        // 读取ToF传感器
        // 注意：根据tof_sensor.h的定义
        // TOF_LEFT = 0 (左侧), TOF_RIGHT = 1 (右侧), TOF_TOP = 2 (顶部/前方)
        g_tof_front = tof_get_top_distance();    // 前方ToF (顶部传感器)
        g_tof_left = tof_get_left_distance();    // 左侧ToF
        g_tof_right = tof_get_right_distance();  // 右侧ToF

        // 读取IMU朝向（Z轴积分）
        static float gyro_z_sum = 0.0f;
        float gyro_z = imu_get_gyro_z();  // 单位: deg/s
        gyro_z_sum += gyro_z * 0.05f;     // 积分（dt = 0.05s）
        g_current_heading = normalize_angle(gyro_z_sum - g_heading_offset);

        // 读取编码器累计距离
        static int32_t last_encoder1 = 0;
        static int32_t last_encoder2 = 0;
        int32_t encoder1 = encoder_get_count();
        int32_t encoder2 = encoder2_get_count();

        // 计算距离增量（轮子直径109mm，编码器12800 counts/rev）
        float wheel_circumference = 0.342f; // m
        int32_t delta1 = encoder1 - last_encoder1;
        int32_t delta2 = encoder2 - last_encoder2;
        float distance_delta = ((delta1 + delta2) / 2.0f) / 12800.0f * wheel_circumference;
        g_total_distance += distance_delta;

        last_encoder1 = encoder1;
        last_encoder2 = encoder2;

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

/* ========== 状态机处理函数 / State Machine Handlers ========== */

/**
 * @brief 状态: 寻找墙壁
 */
static void handle_find_wall(void)
{
    // 原地旋转，寻找右侧墙壁
    chassis_set_velocity(0.0f, WF_TURN_ANGULAR_SPEED);

    // 检测右侧是否有墙
    if (g_tof_right > 0 && g_tof_right < SIDE_WALL_FOUND_THRESHOLD) {
        ESP_LOGI(TAG, "[FIND_WALL] Wall found at right side: %d mm", g_tof_right);

        // 记录当前朝向作为初始朝向
        g_initial_heading = g_current_heading;
        reset_pid();

        // 切换到循墙状态
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_current_state = WF_STATE_FOLLOW_WALL;
            xSemaphoreGive(g_state_mutex);
        }
        chassis_set_velocity(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief 状态: 循墙前进
 */
static void handle_follow_wall(void)
{
    // 检查前方是否有障碍物（凹角）
    if (g_tof_front > 0 && g_tof_front < FRONT_OBSTACLE_THRESHOLD) {
        ESP_LOGI(TAG, "[FOLLOW_WALL] Front obstacle detected: %d mm", g_tof_front);
        ESP_LOGI(TAG, "[FOLLOW_WALL] Switching to TURN_AWAY (concave corner)");

        // 设置目标朝向：左转90度（远离墙壁）
        g_target_heading = normalize_angle(g_current_heading - 90.0f);
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_current_state = WF_STATE_TURN_AWAY;
            xSemaphoreGive(g_state_mutex);
        }
        chassis_set_velocity(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(200));
        return;
    }

    // 检查侧面墙壁是否消失（凸角）
    if (g_tof_right == 0 || g_tof_right > SIDE_WALL_LOST_THRESHOLD) {
        ESP_LOGI(TAG, "[FOLLOW_WALL] Side wall lost: %d mm", g_tof_right);
        ESP_LOGI(TAG, "[FOLLOW_WALL] Switching to CLEARANCE (convex corner)");

        // 记录当前距离，准备延迟直行
        g_clearance_start_distance = g_total_distance;
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_current_state = WF_STATE_CLEARANCE;
            xSemaphoreGive(g_state_mutex);
        }
        return;
    }

    // PID控制循墙距离
    float angular_correction = wall_follow_pid(WALL_DISTANCE_TARGET, g_tof_right, 0.05f);

    // 设置底盘速度：前进 + 角度修正
    chassis_set_velocity(WF_FORWARD_SPEED, angular_correction);
}

/**
 * @brief 状态: 转离墙壁（处理凹角）
 */
static void handle_turn_away(void)
{
    float heading_error = angle_difference(g_target_heading, g_current_heading);

    if (fabs(heading_error) < WF_TURN_ANGLE_TOLERANCE) {
        ESP_LOGI(TAG, "[TURN_AWAY] Turn complete, switching to FOLLOW_WALL");

        chassis_set_velocity(0.0f, 0.0f);
        reset_pid();
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_current_state = WF_STATE_FOLLOW_WALL;
            xSemaphoreGive(g_state_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        // 原地旋转
        float turn_speed = (heading_error > 0) ? WF_TURN_ANGULAR_SPEED : -WF_TURN_ANGULAR_SPEED;
        chassis_set_velocity(0.0f, turn_speed);
    }
}

/**
 * @brief 状态: 延迟直行（绕过凸角）
 */
static void handle_clearance(void)
{
    float distance_traveled = g_total_distance - g_clearance_start_distance;

    if (distance_traveled >= WF_CLEARANCE_DISTANCE) {
        ESP_LOGI(TAG, "[CLEARANCE] Clearance complete, switching to TURN_TOWARD");

        // 设置目标朝向：右转90度（转向墙壁）
        g_target_heading = normalize_angle(g_current_heading + 90.0f);
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_current_state = WF_STATE_TURN_TOWARD;
            xSemaphoreGive(g_state_mutex);
        }
        chassis_set_velocity(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        // 继续直行
        chassis_set_velocity(WF_FORWARD_SPEED, 0.0f);
    }
}

/**
 * @brief 状态: 转向墙壁（寻找墙壁）
 */
static void handle_turn_toward(void)
{
    float heading_error = angle_difference(g_target_heading, g_current_heading);

    if (fabs(heading_error) < WF_TURN_ANGLE_TOLERANCE) {
        ESP_LOGI(TAG, "[TURN_TOWARD] Turn complete, switching to FIND_WALL");

        chassis_set_velocity(0.0f, 0.0f);
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_current_state = WF_STATE_FIND_WALL;
            xSemaphoreGive(g_state_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        // 原地旋转
        float turn_speed = (heading_error > 0) ? WF_TURN_ANGULAR_SPEED : -WF_TURN_ANGULAR_SPEED;
        chassis_set_velocity(0.0f, turn_speed);
    }
}

/* ========== 主控制任务 / Main Control Task ========== */

/**
 * @brief 寻墙主控制任务
 */
static void wall_follow_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50); // 20Hz

    ESP_LOGI(TAG, "Wall following task started");

    while (1) {
        wall_follow_state_t current_state;

        // 读取当前状态
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_state = g_current_state;
            xSemaphoreGive(g_state_mutex);
        } else {
            current_state = WF_STATE_IDLE;
        }

        // 状态机
        switch (current_state) {
            case WF_STATE_FIND_WALL:
                handle_find_wall();
                break;

            case WF_STATE_FOLLOW_WALL:
                handle_follow_wall();
                break;

            case WF_STATE_TURN_AWAY:
                handle_turn_away();
                break;

            case WF_STATE_CLEARANCE:
                handle_clearance();
                break;

            case WF_STATE_TURN_TOWARD:
                handle_turn_toward();
                break;

            case WF_STATE_IDLE:
            case WF_STATE_STOPPED:
                chassis_set_velocity(0.0f, 0.0f);
                break;
        }

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

/* ========== API函数实现 / API Function Implementation ========== */

esp_err_t wall_following_init(void)
{
    if (g_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing wall following system...");

    // 只创建互斥锁，不创建任务（节省内存）
    // 任务将在wall_following_start()时创建
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "Wall following system initialized (tasks will be created on start)");
    ESP_LOGI(TAG, "  Robot collision radius: %d mm", ROBOT_COLLISION_RADIUS_MM);
    ESP_LOGI(TAG, "  ToF side spacing: %d mm", TOF_SIDE_SPACING_MM);
    ESP_LOGI(TAG, "  Wall distance target: %d mm", WALL_DISTANCE_TARGET);

    return ESP_OK;
}

esp_err_t wall_following_start(void)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_FAIL;
    }

    // 检查任务是否已经创建
    if (g_sensor_update_task_handle != NULL || g_wall_follow_task_handle != NULL) {
        ESP_LOGW(TAG, "Wall following already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting wall following...");

    // 创建传感器更新任务 (Core 0, 优先级3)
    BaseType_t ret = xTaskCreatePinnedToCore(
        sensor_update_task,
        "wf_sensor",
        4096,
        NULL,
        3,
        &g_sensor_update_task_handle,
        0
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor update task");
        return ESP_FAIL;
    }

    // 创建寻墙控制任务 (Core 1, 优先级5)
    ret = xTaskCreatePinnedToCore(
        wall_follow_task,
        "wf_control",
        4096,
        NULL,
        5,
        &g_wall_follow_task_handle,
        1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create wall follow task");
        // 清理已创建的传感器任务
        if (g_sensor_update_task_handle != NULL) {
            vTaskDelete(g_sensor_update_task_handle);
            g_sensor_update_task_handle = NULL;
        }
        return ESP_FAIL;
    }

    // 重置状态
    g_total_distance = 0.0f;
    g_heading_offset = 0.0f;
    reset_pid();

    // 设置状态为寻找墙壁
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_current_state = WF_STATE_FIND_WALL;
        g_is_running = true;
        xSemaphoreGive(g_state_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Wall following started (tasks created)");
    return ESP_OK;
}

esp_err_t wall_following_stop(void)
{
    if (!g_initialized) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Stopping wall following...");

    // 设置状态为停止
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_current_state = WF_STATE_STOPPED;
        g_is_running = false;
        xSemaphoreGive(g_state_mutex);
    }

    // 停止底盘
    chassis_set_velocity(0.0f, 0.0f);

    // 等待一小段时间让任务完成当前循环
    vTaskDelay(pdMS_TO_TICKS(100));

    // 删除任务以释放内存
    if (g_sensor_update_task_handle != NULL) {
        vTaskDelete(g_sensor_update_task_handle);
        g_sensor_update_task_handle = NULL;
        ESP_LOGI(TAG, "Sensor update task deleted");
    }

    if (g_wall_follow_task_handle != NULL) {
        vTaskDelete(g_wall_follow_task_handle);
        g_wall_follow_task_handle = NULL;
        ESP_LOGI(TAG, "Wall follow task deleted");
    }

    ESP_LOGI(TAG, "Wall following stopped (tasks deleted, memory freed)");
    return ESP_OK;
}

esp_err_t wall_following_get_status(wall_follow_status_t *status)
{
    if (!g_initialized || status == NULL) {
        return ESP_FAIL;
    }

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        status->state = g_current_state;
        status->is_running = g_is_running;
        xSemaphoreGive(g_state_mutex);
    }

    status->tof_front = g_tof_front;
    status->tof_left = g_tof_left;
    status->tof_right = g_tof_right;
    status->current_heading = g_current_heading;
    status->target_heading = g_target_heading;
    status->total_distance = g_total_distance;

    return ESP_OK;
}

bool wall_following_is_running(void)
{
    bool running = false;

    if (g_initialized && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        running = g_is_running;
        xSemaphoreGive(g_state_mutex);
    }

    return running;
}

