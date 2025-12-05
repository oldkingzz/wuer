/**
 * @file wall_following_test.ino
 * @brief 差速轮机器人寻墙算法测试程序
 * Wall Following Algorithm Test for Differential Drive Robot
 *
 * 传感器配置 / Sensor Configuration:
 * - 3个VL53L0X ToF传感器（前、左、右）
 * - 1个MPU6050 IMU（用于精确转向）
 * - 2个电机编码器（差速驱动）
 *
 * 算法特性 / Algorithm Features:
 * - 状态机控制（State Machine）
 * - 处理凹角（墙角）和凸角（Tower/Nexus）
 * - 原地自旋转向（Spin in Place）
 * - PID循墙控制
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// 包含必要的模块
#include <Wire.h>
#include "src/include/gpio_config.h"
#include "src/include/motor_driver.h"
#include "src/include/encoder.h"
#include "src/include/chassis.h"
#include "src/include/tof_sensor.h"
#include "src/include/imu_sensor.h"

static const char *TAG = "WALL_FOLLOW";

// ========================================
// 用户可配置参数 / User Configurable Parameters
// ========================================
#define WALL_DISTANCE_TARGET        200     // [建议] 20cm (你的半径约14cm + 6cm安全余量)
#define FRONT_OBSTACLE_THRESHOLD    220     // [建议] 22cm (必须给车头预留刹车和旋转的空间)
#define SIDE_WALL_LOST_THRESHOLD    450     // [建议] >45cm 视为凸角 (Nexus/Tower结束)
#define SIDE_WALL_FOUND_THRESHOLD   350     // [建议] <35cm 视为重新找到墙

// --- 传感器物理布局 (单位: mm) ---
// 用于检测是否平行于墙壁
#define TOF_SIDE_SPACING            150     // 【!!!请务必用尺子量准后修改这里!!!】 左右ToF的真实间距

// --- 运动参数 ---
#define FORWARD_SPEED               0.20f   // [修改] 提速至 0.2 m/s (大车太慢容易推不动)
#define TURN_ANGULAR_SPEED          1.5f    // [修改] 提速至 1.5 rad/s (约90度/秒，转太慢IMU积分误差大)
#define CLEARANCE_DISTANCE          0.32f   // [!!!关键修改!!!] 0.32m (32cm)
                                            // 你的车长28cm，必须走过32cm才能保证车尾完全甩开障碍物，
                                            // 否则右转时屁股必撞 Tower。

// --- PID参数（循墙距离控制） ---
#define WALL_FOLLOW_KP              0.003f  // [修改] 增大KP (0.001对大车太软，修正不回来)
#define WALL_FOLLOW_KI              0.0f    // 积分增益 (通常不需要)
#define WALL_FOLLOW_KD              0.005f  // [修改] 增大KD (增加阻尼，防止大车身左右晃动)

// --- IMU转向精度 ---
#define TURN_ANGLE_TOLERANCE        2.0f    // 转向角度容差 (度)
// ========================================
// 状态机定义 / State Machine Definition
// ========================================

typedef enum {
    STATE_FIND_WALL,        // 寻找墙壁
    STATE_FOLLOW_WALL,      // 循墙前进
    STATE_TURN_AWAY,        // 遇到凹角，转离墙壁
    STATE_TURN_TOWARD,      // 遇到凸角，转向墙壁
    STATE_CLEARANCE,        // 绕过凸角后的延迟直行
    STATE_STOPPED           // 停止
} wall_follow_state_t;

// ========================================
// 全局变量 / Global Variables
// ========================================

static wall_follow_state_t g_current_state = STATE_FIND_WALL;
static float g_target_heading = 0.0f;       // 目标朝向 (度)
static float g_initial_heading = 0.0f;      // 初始朝向
static float g_clearance_start_distance = 0.0f; // 延迟直行起始距离

// PID控制器变量
static float g_pid_integral = 0.0f;
static float g_pid_prev_error = 0.0f;

// ToF传感器数据
static uint16_t g_tof_front = 0;
static uint16_t g_tof_left = 0;
static uint16_t g_tof_right = 0;

// IMU数据
static float g_current_heading = 0.0f;      // 当前朝向 (度)
static float g_heading_offset = 0.0f;       // 朝向偏移量（用于归零）

// 编码器累计距离
static float g_total_distance = 0.0f;

// 任务句柄
static TaskHandle_t wall_follow_task_handle = NULL;
static TaskHandle_t sensor_update_task_handle = NULL;

// ========================================
// 辅助函数 / Helper Functions
// ========================================

/**
 * @brief 角度归一化到 [-180, 180]
 */
float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 计算两个角度之间的最小差值
 */
float angle_difference(float target, float current)
{
    float diff = target - current;
    return normalize_angle(diff);
}

/**
 * @brief PID控制器（循墙距离控制）
 */
float wall_follow_pid(float target_distance, float current_distance, float dt)
{
    float error = target_distance - current_distance;

    g_pid_integral += error * dt;
    float derivative = (error - g_pid_prev_error) / dt;
    g_pid_prev_error = error;

    float output = WALL_FOLLOW_KP * error +
                   WALL_FOLLOW_KI * g_pid_integral +
                   WALL_FOLLOW_KD * derivative;

    return output;
}

/**
 * @brief 重置PID控制器
 */
void reset_pid()
{
    g_pid_integral = 0.0f;
    g_pid_prev_error = 0.0f;

// ========================================
// 传感器更新任务 / Sensor Update Task
// ========================================

/**
 * @brief 传感器数据更新任务
 * 周期性读取ToF和IMU数据
 */
static void sensor_update_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50); // 20Hz

    ESP_LOGI(TAG, "Sensor update task started");

    while (1) {
        // 读取ToF传感器
        g_tof_front = tof_read_distance(0);  // 前方ToF
        g_tof_left = tof_read_distance(1);   // 左侧ToF
        g_tof_right = tof_read_distance(2);  // 右侧ToF

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

        // 计算距离增量（假设轮子直径109mm，编码器12800 counts/rev）
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

// ========================================
// 状态机处理函数 / State Machine Handlers
// ========================================

/**
 * @brief 状态: 寻找墙壁
 * 机器人原地旋转，直到侧面ToF检测到墙壁
 */
void handle_find_wall()
{
    ESP_LOGI(TAG, "[FIND_WALL] Searching for wall...");

    // 原地旋转，寻找右侧墙壁
    chassis_set_velocity(0.0f, TURN_ANGULAR_SPEED);

    // 检测右侧是否有墙
    if (g_tof_right > 0 && g_tof_right < SIDE_WALL_FOUND_THRESHOLD) {
        ESP_LOGI(TAG, "[FIND_WALL] Wall found at right side: %d mm", g_tof_right);

        // 记录当前朝向作为初始朝向
        g_initial_heading = g_current_heading;
        reset_pid();

        // 切换到循墙状态
        g_current_state = STATE_FOLLOW_WALL;
        chassis_set_velocity(0.0f, 0.0f); // 停止旋转
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief 状态: 循墙前进
 * 使用PID控制保持与墙壁的距离
 */
void handle_follow_wall()
{
    // 检查前方是否有障碍物（凹角）
    if (g_tof_front > 0 && g_tof_front < FRONT_OBSTACLE_THRESHOLD) {
        ESP_LOGI(TAG, "[FOLLOW_WALL] Front obstacle detected: %d mm", g_tof_front);
        ESP_LOGI(TAG, "[FOLLOW_WALL] Switching to TURN_AWAY (concave corner)");

        // 设置目标朝向：左转90度（远离墙壁）
        g_target_heading = normalize_angle(g_current_heading - 90.0f);
        g_current_state = STATE_TURN_AWAY;
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
        g_current_state = STATE_CLEARANCE;
        return;
    }

    // PID控制循墙距离
    float angular_correction = wall_follow_pid(WALL_DISTANCE_TARGET, g_tof_right, 0.05f);

    // 设置底盘速度：前进 + 角度修正
    chassis_set_velocity(FORWARD_SPEED, angular_correction);

    // 打印调试信息
    static uint32_t print_counter = 0;
    if (++print_counter % 20 == 0) {
        ESP_LOGI(TAG, "[FOLLOW_WALL] Front: %d mm, Right: %d mm, Heading: %.1f°",
                 g_tof_front, g_tof_right, g_current_heading);
    }
}

/**
 * @brief 状态: 转离墙壁（处理凹角）
 * 原地旋转90度，远离墙壁
 */
void handle_turn_away()
{
    float heading_error = angle_difference(g_target_heading, g_current_heading);

    ESP_LOGI(TAG, "[TURN_AWAY] Target: %.1f°, Current: %.1f°, Error: %.1f°",
             g_target_heading, g_current_heading, heading_error);

    if (fabs(heading_error) < TURN_ANGLE_TOLERANCE) {
        ESP_LOGI(TAG, "[TURN_AWAY] Turn complete, switching to FOLLOW_WALL");

        chassis_set_velocity(0.0f, 0.0f);
        reset_pid();
        g_current_state = STATE_FOLLOW_WALL;
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        // 原地旋转
        float turn_speed = (heading_error > 0) ? TURN_ANGULAR_SPEED : -TURN_ANGULAR_SPEED;
        chassis_set_velocity(0.0f, turn_speed);
    }
}

/**
 * @brief 状态: 延迟直行（绕过凸角）
 * 直行一小段距离，确保车身完全越过障碍物
 */
void handle_clearance()
{
    float distance_traveled = g_total_distance - g_clearance_start_distance;

    ESP_LOGI(TAG, "[CLEARANCE] Distance traveled: %.3f m / %.3f m",
             distance_traveled, CLEARANCE_DISTANCE);

    if (distance_traveled >= CLEARANCE_DISTANCE) {
        ESP_LOGI(TAG, "[CLEARANCE] Clearance complete, switching to TURN_TOWARD");

        // 设置目标朝向：右转90度（转向墙壁）
        g_target_heading = normalize_angle(g_current_heading + 90.0f);
        g_current_state = STATE_TURN_TOWARD;
        chassis_set_velocity(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        // 继续直行
        chassis_set_velocity(FORWARD_SPEED, 0.0f);
    }
}

/**
 * @brief 状态: 转向墙壁（寻找墙壁）
 * 原地旋转90度，朝向墙壁
 */
void handle_turn_toward()
{
    float heading_error = angle_difference(g_target_heading, g_current_heading);

    ESP_LOGI(TAG, "[TURN_TOWARD] Target: %.1f°, Current: %.1f°, Error: %.1f°",
             g_target_heading, g_current_heading, heading_error);

    if (fabs(heading_error) < TURN_ANGLE_TOLERANCE) {
        ESP_LOGI(TAG, "[TURN_TOWARD] Turn complete, switching to FIND_WALL");

        chassis_set_velocity(0.0f, 0.0f);
        g_current_state = STATE_FIND_WALL;
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        // 原地旋转
        float turn_speed = (heading_error > 0) ? TURN_ANGULAR_SPEED : -TURN_ANGULAR_SPEED;
        chassis_set_velocity(0.0f, turn_speed);
    }
}

// ========================================
// 主控制任务 / Main Control Task
// ========================================

/**
 * @brief 寻墙主控制任务
 * 20Hz状态机循环
 */
static void wall_follow_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50); // 20Hz

    ESP_LOGI(TAG, "Wall following task started");

    while (1) {
        switch (g_current_state) {
            case STATE_FIND_WALL:
                handle_find_wall();
                break;

            case STATE_FOLLOW_WALL:
                handle_follow_wall();
                break;

            case STATE_TURN_AWAY:
                handle_turn_away();
                break;

            case STATE_CLEARANCE:
                handle_clearance();
                break;

            case STATE_TURN_TOWARD:
                handle_turn_toward();
                break;

            case STATE_STOPPED:
                chassis_set_velocity(0.0f, 0.0f);
                break;
        }

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// ========================================
// Arduino Setup & Loop
// ========================================

void setup()
{
    // 初始化串口
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("Wall Following Test Program");
    Serial.println("========================================");

    // 初始化I2C
    Serial.println("Step 1/7: Initializing I2C bus...");
    Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);
    Wire.setClock(I2C_FREQ_HZ);
    Serial.println("OK: I2C initialized");

    // 初始化电机驱动
    Serial.println("Step 2/7: Initializing motor drivers...");
    esp_err_t ret = motor_driver_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Motor 1 init failed!");
        while(1) { delay(1000); }
    }
    ret = motor2_driver_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Motor 2 init failed!");
        while(1) { delay(1000); }
    }
    Serial.println("OK: Motor drivers initialized");

    // 初始化编码器
    Serial.println("Step 3/7: Initializing encoders...");
    ret = encoder_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Encoder 1 init failed!");
        while(1) { delay(1000); }
    }
    ret = encoder2_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Encoder 2 init failed!");
        while(1) { delay(1000); }
    }
    Serial.println("OK: Encoders initialized");

    // 初始化ToF传感器
    Serial.println("Step 4/7: Initializing ToF sensors...");
    ret = tof_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: ToF init failed!");
        while(1) { delay(1000); }
    }
    Serial.println("OK: ToF sensors initialized");


    // 初始化IMU
    Serial.println("Step 5/7: Initializing IMU...");
    ret = imu_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: IMU init failed!");
        while(1) { delay(1000); }
    }
    Serial.println("OK: IMU initialized");

    // 初始化底盘控制
    Serial.println("Step 6/7: Initializing chassis control...");
    ret = chassis_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Chassis init failed!");
        while(1) { delay(1000); }
    }
    Serial.println("OK: Chassis control initialized");

    // IMU归零（记录初始朝向）
    Serial.println("Step 7/7: Calibrating IMU heading...");
    delay(1000);
    g_heading_offset = 0.0f;
    Serial.println("OK: IMU calibrated");

    Serial.println();
    Serial.println("========================================");
    Serial.println("All modules initialized!");
    Serial.println("========================================");
    Serial.println();

    // 打印配置参数
    Serial.println("Configuration:");
    Serial.printf("  Wall Distance Target: %d mm\n", WALL_DISTANCE_TARGET);
    Serial.printf("  Front Obstacle Threshold: %d mm\n", FRONT_OBSTACLE_THRESHOLD);
    Serial.printf("  Side Wall Lost Threshold: %d mm\n", SIDE_WALL_LOST_THRESHOLD);
    Serial.printf("  Forward Speed: %.2f m/s\n", FORWARD_SPEED);
    Serial.printf("  Turn Angular Speed: %.2f rad/s\n", TURN_ANGULAR_SPEED);
    Serial.printf("  Clearance Distance: %.2f m\n", CLEARANCE_DISTANCE);
    Serial.println();

    // 创建任务
    Serial.println("Creating tasks...");

    // 传感器更新任务 (Core 0, 优先级3)
    xTaskCreatePinnedToCore(
        sensor_update_task,
        "sensor_upd",
        4096,
        NULL,
        3,
        &sensor_update_task_handle,
        0
    );
    Serial.println("  [Core 0] sensor_update_task (Priority 3, 20Hz)");

    // 寻墙控制任务 (Core 1, 优先级5)
    xTaskCreatePinnedToCore(
        wall_follow_task,
        "wall_follow",
        4096,
        NULL,
        5,
        &wall_follow_task_handle,
        1
    );
    Serial.println("  [Core 1] wall_follow_task (Priority 5, 20Hz)");

    Serial.println();
    Serial.println("========================================");
    Serial.println("Wall Following Started!");
    Serial.println("========================================");
    Serial.println();
    Serial.println("State Machine:");
    Serial.println("  FIND_WALL    -> Searching for wall");
    Serial.println("  FOLLOW_WALL  -> Following wall with PID");
    Serial.println("  TURN_AWAY    -> Turning away from wall (concave corner)");
    Serial.println("  CLEARANCE    -> Moving forward to clear obstacle");
    Serial.println("  TURN_TOWARD  -> Turning toward wall (convex corner)");
    Serial.println();
}

void loop()
{
    // 主循环空闲，所有控制由FreeRTOS任务处理
    // 这里可以添加串口命令处理等功能

    static uint32_t last_status_print = 0;
    if (millis() - last_status_print > 2000) {
        last_status_print = millis();

        const char* state_str;
        switch (g_current_state) {
            case STATE_FIND_WALL:   state_str = "FIND_WALL"; break;
            case STATE_FOLLOW_WALL: state_str = "FOLLOW_WALL"; break;
            case STATE_TURN_AWAY:   state_str = "TURN_AWAY"; break;
            case STATE_CLEARANCE:   state_str = "CLEARANCE"; break;
            case STATE_TURN_TOWARD: state_str = "TURN_TOWARD"; break;
            case STATE_STOPPED:     state_str = "STOPPED"; break;
            default:                state_str = "UNKNOWN"; break;
        }

        Serial.println("========================================");
        Serial.printf("State: %s\n", state_str);
        Serial.printf("ToF - Front: %d mm, Left: %d mm, Right: %d mm\n",
                      g_tof_front, g_tof_left, g_tof_right);
        Serial.printf("IMU - Heading: %.1f°\n", g_current_heading);
        Serial.printf("Distance Traveled: %.2f m\n", g_total_distance);
        Serial.println("========================================");
    }

    delay(100);
}
