/**
 * @file vive_navigation.cpp
 * @brief A* Path Planning Navigation System Implementation
 */

#include "include/vive_navigation.h"
#include "include/vive_sensor.h"
#include "include/chassis.h"
#include "include/astar.h"
#include "include/grid_map.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "VIVE_NAV";

// Navigation state variables
static nav_state_t g_nav_state = NAV_STATE_IDLE;
static vive_point_t g_target_vive = {0, 0};
static map_point_t g_target_map = {0, 0};
static vive_pose_t g_current_pose = {0, 0, 0.0f, false};
static map_point_t g_current_map_pos = {0, 0};
static bool g_target_set = false;
static bool g_nav_initialized = false;

// Path planning
static path_t g_current_path;
static uint16_t g_current_waypoint_index = 0;

// Mutex
static SemaphoreHandle_t g_nav_mutex = NULL;

// Navigation task handle
static TaskHandle_t g_nav_task_handle = NULL;

// Math constants
#define PI 3.14159265359f
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / PI)

static float calculate_distance_pixels(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    float dx = (float)(x2 - x1);
    float dy = (float)(y2 - y1);
    return sqrtf(dx * dx + dy * dy);
}

static float calculate_angle_pixels(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    float dx = (float)(x2 - x1);
    float dy = (float)(y2 - y1);
    float angle_rad = atan2f(dy, dx);
    float angle_deg = RAD_TO_DEG(angle_rad);
    if (angle_deg < 0) angle_deg += 360.0f;
    return angle_deg;
}

static float angle_difference(float target_angle, float current_angle)
{
    float diff = target_angle - current_angle;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

static esp_err_t update_robot_pose(void)
{
    vive_data_t vive1, vive2;
    vive_read_all(&vive1, &vive2);

    if (!vive1.valid || !vive2.valid) {
        g_current_pose.valid = false;
        return ESP_ERR_INVALID_STATE;
    }

    g_current_pose.x = (vive1.x + vive2.x) / 2;
    g_current_pose.y = (vive1.y + vive2.y) / 2;

    float dx = (float)((int32_t)vive2.x - (int32_t)vive1.x);
    float dy = (float)((int32_t)vive2.y - (int32_t)vive1.y);
    float angle_rad = atan2f(dy, dx);
    g_current_pose.heading = RAD_TO_DEG(angle_rad);
    if (g_current_pose.heading < 0) g_current_pose.heading += 360.0f;
    g_current_pose.valid = true;

    grid_map_vive_to_pixel(g_current_pose.x, g_current_pose.y,
                          &g_current_map_pos.x, &g_current_map_pos.y);
    return ESP_OK;
}

static esp_err_t plan_path(void)
{
    ESP_LOGI(TAG, "Planning path from (%d,%d) to (%d,%d)",
             g_current_map_pos.x, g_current_map_pos.y, g_target_map.x, g_target_map.y);

    esp_err_t ret = astar_plan_path(g_current_map_pos.x, g_current_map_pos.y,
                                    g_target_map.x, g_target_map.y, &g_current_path);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Path planning failed!");
        return ESP_FAIL;
    }

    astar_simplify_path(&g_current_path);
    ESP_LOGI(TAG, "Path planned: %d waypoints", g_current_path.length);
    g_current_waypoint_index = 0;
    return ESP_OK;
}


static void navigation_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(50);  // 20Hz

    ESP_LOGI(TAG, "Navigation task started");

    while (1) {
        if (xSemaphoreTake(g_nav_mutex, portMAX_DELAY) == pdTRUE) {
            update_robot_pose();

            switch (g_nav_state) {
                case NAV_STATE_IDLE:
                    chassis_set_velocity(0.0f, 0.0f);
                    break;

                case NAV_STATE_PLANNING:
                {
                    ESP_LOGI(TAG, "Starting path planning...");
                    UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(NULL);
                    ESP_LOGI(TAG, "Stack free before planning: %u bytes", stack_high_water * 4);

                    if (plan_path() == ESP_OK) {
                        g_nav_state = NAV_STATE_NAVIGATING;
                        ESP_LOGI(TAG, "Switched to NAVIGATING");
                    } else {
                        g_nav_state = NAV_STATE_ERROR;
                        ESP_LOGE(TAG, "Planning failed");
                    }

                    stack_high_water = uxTaskGetStackHighWaterMark(NULL);
                    ESP_LOGI(TAG, "Stack free after planning: %u bytes", stack_high_water * 4);
                    break;
                }

                case NAV_STATE_NAVIGATING:
                {
                    if (!g_current_pose.valid) {
                        chassis_set_velocity(0.0f, 0.0f);
                        break;
                    }

                    float dist_to_goal = calculate_distance_pixels(
                        g_current_map_pos.x, g_current_map_pos.y,
                        g_target_map.x, g_target_map.y);

                    if (dist_to_goal < NAV_ARRIVAL_THRESHOLD) {
                        ESP_LOGI(TAG, "Arrived!");
                        g_nav_state = NAV_STATE_ARRIVED;
                        chassis_set_velocity(0.0f, 0.0f);
                        break;
                    }

                    int16_t target_x, target_y;
                    if (astar_get_next_target(&g_current_path,
                                             g_current_map_pos.x, g_current_map_pos.y,
                                             NAV_LOOKAHEAD_DISTANCE,
                                             &target_x, &target_y) != ESP_OK) {
                        chassis_set_velocity(0.0f, 0.0f);
                        break;
                    }

                    float distance = calculate_distance_pixels(
                        g_current_map_pos.x, g_current_map_pos.y, target_x, target_y);

                    float target_heading = calculate_angle_pixels(
                        g_current_map_pos.x, g_current_map_pos.y, target_x, target_y);

                    float heading_error = angle_difference(target_heading, g_current_pose.heading);

                    // 简单比例控制（不使用PID，因为PID需要速度反馈）
                    // 距离越大，速度越快
                    float linear_velocity = distance * NAV_DISTANCE_KP;
                    // 角度误差越大，转向越快
                    float angular_velocity = heading_error * NAV_HEADING_KP;

                    if (linear_velocity > NAV_MAX_LINEAR_VELOCITY) linear_velocity = NAV_MAX_LINEAR_VELOCITY;
                    if (linear_velocity < 0.0f) linear_velocity = 0.0f;
                    if (angular_velocity > NAV_MAX_ANGULAR_VELOCITY) angular_velocity = NAV_MAX_ANGULAR_VELOCITY;
                    if (angular_velocity < -NAV_MAX_ANGULAR_VELOCITY) angular_velocity = -NAV_MAX_ANGULAR_VELOCITY;

                    chassis_set_velocity(linear_velocity, angular_velocity);

                    static uint32_t replan_check_counter = 0;
                    if (++replan_check_counter >= 40) {
                        replan_check_counter = 0;
                        float min_dist_to_path = 1e9f;
                        for (uint16_t i = 0; i < g_current_path.length; i++) {
                            float d = calculate_distance_pixels(
                                g_current_map_pos.x, g_current_map_pos.y,
                                g_current_path.waypoints[i].x, g_current_path.waypoints[i].y);
                            if (d < min_dist_to_path) min_dist_to_path = d;
                        }
                        if (min_dist_to_path > NAV_REPLAN_THRESHOLD) {
                            ESP_LOGW(TAG, "Replanning (dist=%.1f)", min_dist_to_path);
                            g_nav_state = NAV_STATE_PLANNING;
                        }
                    }
                    break;
                }

                case NAV_STATE_ARRIVED:
                case NAV_STATE_ERROR:
                    chassis_set_velocity(0.0f, 0.0f);
                    break;
            }

            xSemaphoreGive(g_nav_mutex);
        }
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

esp_err_t vive_nav_init(void)
{
    if (g_nav_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing navigation...");
    ESP_LOGI(TAG, "Step 1: Calling grid_map_init()...");

    if (grid_map_init() != ESP_OK) {
        ESP_LOGE(TAG, "Grid map init failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Step 1: grid_map_init() completed");
    ESP_LOGI(TAG, "Step 2: Calling astar_init()...");

    if (astar_init() != ESP_OK) {
        ESP_LOGE(TAG, "A* init failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Step 2: astar_init() completed");
    ESP_LOGI(TAG, "Step 3: Creating navigation mutex...");

    g_nav_mutex = xSemaphoreCreateMutex();
    if (g_nav_mutex == NULL) {
        ESP_LOGE(TAG, "Mutex creation failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Step 3: Mutex created");

    // 不在初始化时创建任务，避免任务立即运行并调用vive_read_all()
    // 任务将在第一次调用vive_nav_set_target()时创建
    g_nav_task_handle = NULL;

    g_nav_initialized = true;
    ESP_LOGI(TAG, "Navigation initialized successfully (task will be created on first use)");
    return ESP_OK;
}


esp_err_t vive_nav_set_target(uint16_t target_x, uint16_t target_y)
{
    if (!g_nav_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_target_vive.x = target_x;
        g_target_vive.y = target_y;

        grid_map_vive_to_pixel(target_x, target_y, &g_target_map.x, &g_target_map.y);

        ESP_LOGI(TAG, "Target set: Vive(%d,%d) -> Map(%d,%d)",
                 target_x, target_y, g_target_map.x, g_target_map.y);

        g_target_set = true;
        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t vive_nav_set_target_map(int16_t map_x, int16_t map_y)
{
    if (!g_nav_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_target_map.x = map_x;
        g_target_map.y = map_y;

        grid_map_pixel_to_vive(map_x, map_y, &g_target_vive.x, &g_target_vive.y);

        ESP_LOGI(TAG, "Target set: Map(%d,%d) -> Vive(%d,%d)",
                 map_x, map_y, g_target_vive.x, g_target_vive.y);

        g_target_set = true;
        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t vive_nav_start(void)
{
    if (!g_nav_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_target_set) {
        ESP_LOGE(TAG, "No target set");
        return ESP_ERR_INVALID_STATE;
    }

    // 如果任务还没创建，现在创建它
    if (g_nav_task_handle == NULL) {
        ESP_LOGI(TAG, "Creating navigation task...");
        ESP_LOGI(TAG, "Free heap before task creation: %u bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));

        // 栈大小设为6KB，应该足够（A*算法不使用递归）
        BaseType_t ret = xTaskCreatePinnedToCore(
            navigation_task, "nav_task", 6144, NULL, 5, &g_nav_task_handle, 1);

        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Task creation failed");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Navigation task created (stack: 6KB)");
        ESP_LOGI(TAG, "Free heap after task creation: %u bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_nav_state = NAV_STATE_PLANNING;
        ESP_LOGI(TAG, "Navigation started");
        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t vive_nav_stop(void)
{
    if (!g_nav_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_nav_state = NAV_STATE_IDLE;
        chassis_set_velocity(0.0f, 0.0f);
        ESP_LOGI(TAG, "Navigation stopped");
        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t vive_nav_get_status(nav_status_t *status)
{
    if (!g_nav_initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status->state = g_nav_state;
        status->current_pose = g_current_pose;
        status->target = g_target_vive;
        status->current_map_pos = g_current_map_pos;
        status->target_map_pos = g_target_map;
        status->path_length = g_current_path.length;
        status->current_waypoint = g_current_waypoint_index;

        status->distance_to_target = calculate_distance_pixels(
            g_current_map_pos.x, g_current_map_pos.y,
            g_target_map.x, g_target_map.y);

        float target_heading = calculate_angle_pixels(
            g_current_map_pos.x, g_current_map_pos.y,
            g_target_map.x, g_target_map.y);

        status->heading_error = angle_difference(target_heading, g_current_pose.heading);
        status->linear_velocity = 0.0f;  // TODO: get from chassis
        status->angular_velocity = 0.0f;

        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

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

esp_err_t vive_nav_get_path(path_t *path)
{
    if (!g_nav_initialized || path == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *path = g_current_path;
        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t vive_nav_replan(void)
{
    if (!g_nav_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_nav_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_nav_state == NAV_STATE_NAVIGATING) {
            g_nav_state = NAV_STATE_PLANNING;
            ESP_LOGI(TAG, "Replanning requested");
        }
        xSemaphoreGive(g_nav_mutex);
        return ESP_OK;
    }

    return ESP_FAIL;
}
