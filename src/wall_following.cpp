/**
 * @file wall_following.cpp
 * @brief 寻墙算法实现 - 沿地图边缘行驶
 */

#include "include/wall_following.h"
#include "include/chassis_v2.h"  // 使用新的Chassis V2
#include "include/tof_sensor.h"
#include "include/imu_sensor.h"
#include "include/encoder.h"
#include "include/robot_config.h" // 统一线速度配置
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "WALL_FOLLOW";

/* ========== 常量 ========== */

#define PI 3.14159265359f
#define MM_TO_INCHES(mm) ((mm) / 25.4f)

// 地图尺寸（英寸）
#define MAP_WIDTH   60.0f
#define MAP_HEIGHT  144.0f

// 车辆参数（与chassis_v2.cpp保持一致）
#define ROBOT_RADIUS              5.0f     // 碰撞半径（英寸）
#define CHASSIS_WHEEL_DIAMETER_M  0.065f   // 轮径 (m)
#define CHASSIS_WHEEL_BASE_M      0.15f    // 轮距 (m)

// 轨迹/安全参数
#define SAFE_MARGIN_INCH     3.0f                       // 车中心到墙的安全余量
#define WALL_CLEARANCE       (ROBOT_RADIUS + SAFE_MARGIN_INCH)  // 车中心距墙的目标距离 = 8"

// 控制参数
// 前进速度统一使用 ROBOT_BASE_LINEAR_SPEED，方便全局调整
#define FORWARD_SPEED        ROBOT_BASE_LINEAR_SPEED   // 前进速度 (m/s)
#define TURN_SPEED           1.0f    // 最大转向速度 (rad/s)
#define WALL_TARGET_DIST     80.0f   // 目标墙距 (mm) - 仅用于旧的循墙PID
#define PID_KP               0.01f   // PID比例系数

// 轨迹跟随控制参数
#define HEADING_KP           2.0f    // 朝向误差 -> 角速度 (rad/s per rad)
#define WP_REACHED_DIST      3.0f    // 判定到达路径点的距离阈值 (inch)
#define WP_SLOW_DIST         6.0f    // 靠近路径点时减速的距离 (inch)

/* ========== 全局变量 ========== */

static bool g_initialized = false;
static bool g_is_running = false;
static SemaphoreHandle_t g_mutex = NULL;
static TaskHandle_t g_task_handle = NULL;

static wall_follow_state_t g_state = WF_STATE_IDLE;
static uint16_t g_tof_front = 0;
static uint16_t g_tof_left = 0;
static uint16_t g_tof_right = 0;

// 位姿（英寸和度）
static float g_pos_x = 0.0f;
static float g_pos_y = 0.0f;
static float g_heading = 0.0f;
static float g_target_heading = 0.0f;  // 当前目标朝向（度），用于可视化

// 里程计
static float g_gyro_sum = 0.0f;
static float g_heading_offset = 0.0f;
static float g_total_dist = 0.0f;

// 路径点定义（单位：英寸，车中心坐标）
typedef struct {
    float x;
    float y;
} waypoint_t;

// 这个路径与 scripts/visualize_wall_follow_path.py 中的 waypoints 完全一致
static const waypoint_t g_path[] = {
    // 第1段：起点（左下角附近）
    {WALL_CLEARANCE, 15},

    // 第2段：左边缘向上
    {WALL_CLEARANCE, 25},
    {WALL_CLEARANCE, 40},
    {WALL_CLEARANCE, 55},
    {WALL_CLEARANCE, 68},
    {WALL_CLEARANCE, 76},
    {WALL_CLEARANCE, 90},
    {WALL_CLEARANCE, 110},
    {WALL_CLEARANCE, 125},

    // 第3段：左上角转弯
    {WALL_CLEARANCE, MAP_HEIGHT - WALL_CLEARANCE},
    {WALL_CLEARANCE + 3, MAP_HEIGHT - WALL_CLEARANCE},

    // 第4段：上边缘向右（绕开北侧基地）
    {15, MAP_HEIGHT - WALL_CLEARANCE},
    {20, MAP_HEIGHT - WALL_CLEARANCE},

    {21, MAP_HEIGHT - WALL_CLEARANCE},
    {21, 134},
    {26, 134},
    {30, 134},
    {34, 134},
    {39, 134},
    {39, MAP_HEIGHT - WALL_CLEARANCE},

    {42, MAP_HEIGHT - WALL_CLEARANCE},
    {48, MAP_HEIGHT - WALL_CLEARANCE},

    // 第5段：右上角转弯
    {MAP_WIDTH - WALL_CLEARANCE - 3, MAP_HEIGHT - WALL_CLEARANCE},
    {MAP_WIDTH - WALL_CLEARANCE,     MAP_HEIGHT - WALL_CLEARANCE},
    {MAP_WIDTH - WALL_CLEARANCE,     MAP_HEIGHT - WALL_CLEARANCE - 3},

    // 第6段：右边缘向下
    {MAP_WIDTH - WALL_CLEARANCE, 125},
    {MAP_WIDTH - WALL_CLEARANCE, 110},
    {MAP_WIDTH - WALL_CLEARANCE, 90},
    {MAP_WIDTH - WALL_CLEARANCE, 76},
    {MAP_WIDTH - WALL_CLEARANCE, 68},
    {MAP_WIDTH - WALL_CLEARANCE, 55},
    {MAP_WIDTH - WALL_CLEARANCE, 40},
    {MAP_WIDTH - WALL_CLEARANCE, 25},
    {MAP_WIDTH - WALL_CLEARANCE, 15},

    // 第7段：右下角转弯
    {MAP_WIDTH - WALL_CLEARANCE,     WALL_CLEARANCE + 3},
    {MAP_WIDTH - WALL_CLEARANCE,     WALL_CLEARANCE},
    {MAP_WIDTH - WALL_CLEARANCE - 3, WALL_CLEARANCE},

    // 第8段：下边缘向左（绕开南侧基地）
    {48, WALL_CLEARANCE},
    {42, WALL_CLEARANCE},

    {39, WALL_CLEARANCE},
    {39, 10},
    {34, 10},
    {30, 10},
    {26, 10},
    {21, 10},
    {21, WALL_CLEARANCE},

    {20, WALL_CLEARANCE},
    {15, WALL_CLEARANCE},

    // 第9段：左下角转弯并回到起点
    {WALL_CLEARANCE + 3, WALL_CLEARANCE},
    {WALL_CLEARANCE,     WALL_CLEARANCE},
    {WALL_CLEARANCE,     WALL_CLEARANCE + 3},
    {WALL_CLEARANCE,     15},
};

static const int g_num_waypoints = sizeof(g_path) / sizeof(g_path[0]);

// 当前所在的路径点索引，以及一圈是否已经开始
static int g_current_wp_idx = 0;
static int g_start_wp_idx   = 0;
static bool g_lap_started   = false;

/* ========== 辅助函数 ========== */

static float normalize_angle(float angle)
{
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/**
 * @brief 计算初始位置
 *
 * 根据 ToF 读数计算起始位置
 * 假设：车在右上角，右侧有墙，前方有墙，车头朝向 +Y
 */
static esp_err_t calc_init_pos(float *x, float *y, float *heading)
{
    ESP_LOGI(TAG, "Calculating initial position (front + right ToF)...");

    // 采样10次取平均
    uint32_t right_sum = 0, front_sum = 0;
    int valid = 0;

    for (int i = 0; i < 10; i++) {
        uint16_t right = tof_get_right_distance();      // 使用已有接口，不改底层
        uint16_t front = tof_get_cached_front_distance();

        if (right > 0 && right < 1000 && front > 0 && front < 1000) {
            right_sum += right;
            front_sum += front;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (valid < 3) {
        ESP_LOGE(TAG, "Not enough valid ToF samples");
        return ESP_FAIL;
    }

    float avg_right = (float)right_sum / valid;
    float avg_front = (float)front_sum / valid;

    ESP_LOGI(TAG, "ToF avg: Right=%.1fmm, Front=%.1fmm", avg_right, avg_front);

    // 右侧和前方均有墙：
    //  - 车中心到右侧墙距离 = MM_TO_INCHES(avg_right) + ROBOT_RADIUS
    //  - 车中心到上边墙距离 = MM_TO_INCHES(avg_front) + ROBOT_RADIUS
    *x = MAP_WIDTH  - MM_TO_INCHES(avg_right) - ROBOT_RADIUS;
    *y = MAP_HEIGHT - MM_TO_INCHES(avg_front) - ROBOT_RADIUS;
    *heading = 90.0f;  // 车头朝向 +Y

    ESP_LOGI(TAG, "Init pos: (%.2f, %.2f), heading=%.1f°", *x, *y, *heading);

    return ESP_OK;
}

/**
 * @brief PID循墙控制
 */
static float wall_follow_pid(float target, float current)
{
    float error = target - current;
    return PID_KP * error;
}

/**
 * @brief 更新里程计
 */
static void update_odometry(float dt)
{
    // 读取编码器
    float left_rpm = encoder2_get_rpm();
    float right_rpm = encoder_get_rpm();

    // 计算速度
    const float wheel_circ = CHASSIS_WHEEL_DIAMETER_M * PI;
    float v_left = (left_rpm / 60.0f) * wheel_circ;
    float v_right = (right_rpm / 60.0f) * wheel_circ;
    float v = (v_left + v_right) / 2.0f;

    // 更新朝向（IMU）
    float gyro_z = imu_get_gyro_z();
    g_gyro_sum += gyro_z * dt;
    g_heading = normalize_angle(g_gyro_sum - g_heading_offset);

    // 更新位置
    float theta_rad = g_heading * PI / 180.0f;
    float dx = v * cosf(theta_rad) * dt;
    float dy = v * sinf(theta_rad) * dt;

    g_pos_x += dx * 39.3701f;  // 米转英寸
    g_pos_y += dy * 39.3701f;

    g_total_dist += sqrtf(dx*dx + dy*dy);
}

/**
 * @brief ToF校正位置
 */
static void correct_position_with_tof(void)
{
    // 如果检测到右侧墙，利用右侧 ToF 对 X 坐标做轻微校正
    if (g_tof_right > 0 && g_tof_right < 500) {
        // 车中心到右墙的理论距离：MM_TO_INCHES(g_tof_right) + ROBOT_RADIUS
        float measured_x = MAP_WIDTH - MM_TO_INCHES(g_tof_right) - ROBOT_RADIUS;
        // 平滑融合，避免抖动
        g_pos_x = g_pos_x * 0.7f + measured_x * 0.3f;
    }
}

/**
 * @brief 找到距离当前位姿最近的路径点索引
 */
static int find_closest_waypoint(float x, float y)
{
    if (g_num_waypoints <= 0) {
        return 0;
    }

    int best_idx = 0;
    float best_dist2 = 1e9f;

    for (int i = 0; i < g_num_waypoints; i++) {
        float dx = g_path[i].x - x;
        float dy = g_path[i].y - y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best_dist2) {
            best_dist2 = d2;
            best_idx = i;
        }
    }
    return best_idx;
}

/**
 * @brief 轨迹跟随一步（根据当前位姿追踪 g_path[g_current_wp_idx]）
 */
static void follow_path_step(void)
{
    if (g_num_waypoints <= 0) {
        chassis_v2_set_velocity(0.0f, 0.0f);
        return;
    }

    waypoint_t target = g_path[g_current_wp_idx];

    float dx = target.x - g_pos_x;
    float dy = target.y - g_pos_y;
    float dist = sqrtf(dx * dx + dy * dy);

    // 到达当前路径点：切换到下一个
    if (dist < WP_REACHED_DIST) {
        int next_idx = (g_current_wp_idx + 1) % g_num_waypoints;

        if (!g_lap_started) {
            // 离开起点后标记一圈已开始
            g_lap_started = true;
        } else if (next_idx == g_start_wp_idx) {
            // 已经绕场一圈，回到起点附近，自动停止
            ESP_LOGI(TAG, "[FOLLOW] Completed one full lap, stopping.");

            chassis_v2_set_velocity(0.0f, 0.0f);
            if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_state = WF_STATE_STOPPED;
                g_is_running = false;
                xSemaphoreGive(g_mutex);
            }
            return;
        }

        g_current_wp_idx = next_idx;
        target = g_path[g_current_wp_idx];
        dx = target.x - g_pos_x;
        dy = target.y - g_pos_y;
        dist = sqrtf(dx * dx + dy * dy);
    }

    // 计算目标朝向和朝向误差
    float target_heading = atan2f(dy, dx) * 180.0f / PI;  // 度
    float heading_error = target_heading - g_heading;
    // 归一化到 [-180, 180]
    while (heading_error > 180.0f) heading_error -= 360.0f;
    while (heading_error < -180.0f) heading_error += 360.0f;

    g_target_heading = target_heading;

    float heading_error_rad = heading_error * PI / 180.0f;
    float angular = HEADING_KP * heading_error_rad;

    // 限制最大角速度
    if (angular > TURN_SPEED) angular = TURN_SPEED;
    if (angular < -TURN_SPEED) angular = -TURN_SPEED;

    // 根据朝向误差和距离选择线速度
    float linear = FORWARD_SPEED;
    if (fabsf(heading_error) > 80.0f) {
        // 误差太大时先原地旋转对准
        linear = 0.0f;
    } else if (dist < WP_SLOW_DIST) {
        // 接近路径点时减速，这里使用基础速度的一半，始终与全局速度联动
        linear = ROBOT_BASE_LINEAR_SPEED * 0.5f;
    }

    chassis_v2_set_velocity(linear, angular);

    // 调试输出：每1秒打印一次
    static int log_count = 0;
    if (++log_count >= 20) {
        ESP_LOGI(TAG,
                 "[FOLLOW] WP %d/%d  Pos:(%.1f,%.1f) Head:%.1f° -> %.1f°  dist=%.1f  v=%.2f w=%.2f",
                 g_current_wp_idx, g_num_waypoints,
                 g_pos_x, g_pos_y,
                 g_heading, target_heading,
                 dist, linear, angular);
        log_count = 0;
    }
}

/* ========== 主任务 ========== */

/**
 * @brief 寻墙主任务
 */
static void wall_follow_task(void *param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t freq = pdMS_TO_TICKS(50);  // 20Hz

    ESP_LOGI(TAG, "Wall follow task started");

    while (1) {
        // 读取ToF（使用现有高层接口，不改底层驱动）
        uint16_t raw_front = tof_get_cached_front_distance();
        uint16_t raw_left  = tof_get_cached_left_front_distance();
        uint16_t raw_right = tof_get_right_distance();

        g_tof_front = (raw_front == 0xFFFF) ? 0 : raw_front;
        g_tof_left  = (raw_left  == 0xFFFF) ? 0 : raw_left;
        g_tof_right = (raw_right == 0xFFFF) ? 0 : raw_right;

        // 调试：每2秒输出ToF读数
        static int tof_log_count = 0;
        if (++tof_log_count >= 40) {  // 40 * 50ms = 2秒
            ESP_LOGI(TAG, "[ToF] Front=%u Left=%u Right=%u (raw: F=%u L=%u R=%u)",
                     g_tof_front, g_tof_left, g_tof_right,
                     raw_front, raw_left, raw_right);
            tof_log_count = 0;
        }

        // 更新里程计
        update_odometry(0.05f);

        // ToF校正
        correct_position_with_tof();

        // 获取当前状态
        wall_follow_state_t state;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            state = g_state;
            xSemaphoreGive(g_mutex);
        } else {
            state = WF_STATE_IDLE;
        }

        // 状态机
        switch (state) {
            case WF_STATE_INIT_POS: {
                ESP_LOGI(TAG, "[STATE] INIT_POS - Calculating initial position...");

                // 计算初始位置（右上角，前+右 ToF）
                float x, y, h;
                if (calc_init_pos(&x, &y, &h) == ESP_OK) {
                    g_pos_x = x;
                    g_pos_y = y;
                    g_heading = h;
                    g_heading_offset = g_gyro_sum - h;

                    // 根据当前位姿选一个最近的路径点作为起点
                    g_start_wp_idx   = find_closest_waypoint(g_pos_x, g_pos_y);
                    g_current_wp_idx = g_start_wp_idx;
                    g_lap_started    = false;

                    ESP_LOGI(TAG, "✅ Init complete! Pos:(%.1f, %.1f) Heading:%.1f° -> Start WP %d (%.1f, %.1f)",
                             x, y, h,
                             g_start_wp_idx,
                             g_path[g_start_wp_idx].x, g_path[g_start_wp_idx].y);
                    ESP_LOGI(TAG, "Switching to FOLLOW_WALL (trajectory) state...");

                    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        g_state = WF_STATE_FOLLOW_WALL;
                        xSemaphoreGive(g_mutex);
                    }
                } else {
                    ESP_LOGE(TAG, "❌ Init failed (ToF invalid), retrying in 500ms...");
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                break;
            }

            case WF_STATE_FOLLOW_WALL: {
                // 预定义轨迹巡航：跟随 g_path 一圈
                follow_path_step();
                break;
            }

            case WF_STATE_IDLE:
            case WF_STATE_STOPPED:
                chassis_v2_set_velocity(0.0f, 0.0f);
                break;
        }

        vTaskDelayUntil(&last_wake, freq);
    }
}

/* ========== API实现 ========== */

esp_err_t wall_following_init(void)
{
    if (g_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing wall following...");

    g_mutex = xSemaphoreCreateMutex();
    if (g_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "Wall following initialized");

    return ESP_OK;
}

esp_err_t wall_following_start(void)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_FAIL;
    }

    // 如果任务已经存在，可能是上一圈自动结束后处于 STOPPED 状态，此时支持“重新开始”
    if (g_task_handle != NULL) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (!g_is_running) {
                ESP_LOGI(TAG, "Restarting existing wall-follow task...");
                g_state = WF_STATE_INIT_POS;
                g_is_running = true;
                xSemaphoreGive(g_mutex);
                return ESP_OK;
            }
            xSemaphoreGive(g_mutex);
        }

        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting wall following...");

    // 启动异步ToF读取
    if (tof_start_async_reading() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ToF async reading");
        return ESP_FAIL;
    }

    // 创建任务
    BaseType_t ret = xTaskCreatePinnedToCore(
        wall_follow_task,
        "wall_follow",
        8192,
        NULL,
        5,
        &g_task_handle,
        1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        tof_stop_async_reading();
        return ESP_FAIL;
    }

    // 重置状态
    g_total_dist = 0.0f;
    g_gyro_sum = 0.0f;

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_state = WF_STATE_INIT_POS;
        g_is_running = true;
        xSemaphoreGive(g_mutex);
    }

    ESP_LOGI(TAG, "Wall following started");
    return ESP_OK;
}

esp_err_t wall_following_stop(void)
{
    if (!g_initialized) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Stopping wall following...");

    // 步骤1：设置停止状态，让任务停止控制底盘
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_state = WF_STATE_STOPPED;
        g_is_running = false;
        xSemaphoreGive(g_mutex);
    }

    // 步骤2：等待任务进入停止状态（最多等待200ms）
    vTaskDelay(pdMS_TO_TICKS(200));

    // 步骤3：强制停止底盘
    chassis_v2_set_velocity(0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 步骤4：删除任务
    if (g_task_handle != NULL) {
        vTaskDelete(g_task_handle);
        g_task_handle = NULL;
    }

    // 步骤5：停止ToF异步读取
    tof_stop_async_reading();

    // 步骤6：再次确保底盘停止
    chassis_v2_set_velocity(0.0f, 0.0f);

    ESP_LOGI(TAG, "Wall following stopped");
    return ESP_OK;
}

esp_err_t wall_following_get_status(wall_follow_status_t *status)
{
    if (!g_initialized || status == NULL) {
        return ESP_FAIL;
    }

	    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
	        status->state = g_state;
	        status->is_running = g_is_running;
	        xSemaphoreGive(g_mutex);
	    }
	
	    status->tof_front = g_tof_front;
	    status->tof_left = g_tof_left;
	    status->tof_right = g_tof_right;
	    status->current_heading = g_heading;
	    status->target_heading = g_target_heading;
	    status->total_distance = g_total_dist;
	
	    return ESP_OK;
}

bool wall_following_is_running(void)
{
    return g_is_running;
}
