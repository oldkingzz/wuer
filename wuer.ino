/**
 * @file wuer.ino
 * @brief ESP32差速驱动底盘控制系统 - Arduino版本
 * ESP32 Differential Drive Chassis Control System - Arduino Version
 *
 * 从ESP-IDF迁移到Arduino
 * Migrated from ESP-IDF to Arduino
 *
 * 功能 / Features:
 * - L298N双电机驱动器PWM控制 / Dual L298N motor driver PWM control
 * - 正交编码器脉冲计数和转速测量 / Quadrature encoder pulse counting and speed measurement
 * - Web界面虚拟摇杆控制 / Web interface virtual joystick control
 * - PID速度闭环控制 / PID speed closed-loop control
 * - 差速驱动运动学 / Differential drive kinematics
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

// 包含自定义模块头文件 / Include custom module headers
#include <Wire.h>
#include "src/include/gpio_config.h"
#include "src/include/motor_driver.h"
#include "src/include/encoder.h"
#include "src/include/user_input.h"
#include "src/include/chassis.h"
#include "src/include/tof_sensor.h"
#include "src/include/imu_sensor.h"
#include "src/include/vive_sensor.h"

static const char *TAG = "MAIN";

// 任务句柄 / Task handles
static TaskHandle_t encoder_update_task_handle = NULL;
static TaskHandle_t status_monitor_task_handle = NULL;
static TaskHandle_t chassis_control_task_handle = NULL;
static TaskHandle_t sensor_update_task_handle = NULL;
static TaskHandle_t vive_update_task_handle = NULL;

/**
 * @brief Vive定位更新任务
 * Vive Positioning Update Task
 *
 * 周期性更新Vive定位传感器数据
 * Periodically updates Vive positioning sensor data
 */
static void vive_update_task(void *pvParameters)
{
    Serial.println("Vive update task started");
    Serial.flush();

    vive_data_t vive1_data, vive2_data;

    // Wait a bit for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Vive task: starting position reads");
    Serial.flush();

    while (1) {
        // Read both Vive sensors (safe even if not initialized)
        vive_read_all(&vive1_data, &vive2_data);

        // Delay 20ms (50Hz update rate - balanced between responsiveness and CPU load)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief 传感器更新任务
 * Sensor Update Task
 *
 * 周期性更新ToF和IMU传感器数据
 * Periodically updates ToF and IMU sensor data
 */
static void sensor_update_task(void *pvParameters)
{
    Serial.println("Sensor update task started");
    Serial.flush();

    tof_data_t left_tof, right_tof, top_tof;
    imu_data_t imu_data;

    // Wait a bit for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Sensor task: starting sensor reads");
    Serial.flush();

    while (1) {
        // Read all ToF sensors (safe even if not initialized)
        esp_err_t ret = tof_read_all(&left_tof, &right_tof, &top_tof);

        // Read IMU data (safe even if not initialized)
        ret = imu_read(&imu_data);

        // Delay 50ms (20Hz update rate)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


/**
 * @brief 编码器更新任务
 * Encoder Update Task
 *
 * 周期性更新编码器转速计算
 * Periodically updates encoder speed calculation
 */
static void encoder_update_task(void *pvParameters)
{
    ESP_LOGI(TAG, "编码器更新任务启动 / Encoder update task started");

    while (1) {
        // 更新电机1转速计算 / Update motor 1 speed calculation
        encoder_update_speed();

        // 更新电机2转速计算 / Update motor 2 speed calculation
        encoder2_update_speed();

        // 延迟100ms / Delay 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief 状态监控任务
 * Status Monitoring Task
 *
 * 周期性打印系统状态信息
 * Periodically prints system status information
 */
static void status_monitor_task(void *pvParameters)
{
    Serial.println("Status monitoring task started");

    while (1) {
        // Delay 3 seconds
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Print separator
        Serial.println();
        Serial.println("========== System Status ==========");

        // Print system health
        Serial.println("System Health:");
        Serial.print("  Free Heap: ");
        Serial.print(ESP.getFreeHeap());
        Serial.println(" bytes");
        Serial.print("  Min Free Heap: ");
        Serial.print(ESP.getMinFreeHeap());
        Serial.println(" bytes");

        Serial.println();

        // Get chassis velocity
        chassis_velocity_t chassis_vel;
        chassis_get_velocity(&chassis_vel);

        Serial.println("Chassis Velocity:");
        Serial.print("  Linear: ");
        Serial.print(chassis_vel.linear_velocity, 3);
        Serial.println(" m/s");
        Serial.print("  Angular: ");
        Serial.print(chassis_vel.angular_velocity, 3);
        Serial.println(" rad/s");
        Serial.print("  State: ");
        Serial.println(chassis_vel.is_moving ? "MOVING" : "STOPPED");

        Serial.println();
        Serial.println("Wheel Speeds:");
        Serial.print("  Left Target: ");
        Serial.print(chassis_vel.left_wheel_rpm, 2);
        Serial.println(" RPM");
        Serial.print("  Right Target: ");
        Serial.print(chassis_vel.right_wheel_rpm, 2);
        Serial.println(" RPM");

        // Get actual wheel speeds
        float left_actual = encoder2_get_rpm();   // Motor 2 = Left wheel
        float right_actual = encoder_get_rpm();   // Motor 1 = Right wheel
        Serial.print("  Left Actual: ");
        Serial.print(left_actual, 2);
        Serial.println(" RPM");
        Serial.print("  Right Actual: ");
        Serial.print(right_actual, 2);
        Serial.println(" RPM");

        Serial.println();
        Serial.println("Encoder Data:");

        // Left wheel encoder
        encoder_data_t left_encoder;
        encoder2_get_data(&left_encoder);
        Serial.print("  Left Pulses: ");
        Serial.println(left_encoder.pulse_count);
        Serial.print("  Left Revolutions: ");
        Serial.println(left_encoder.revolutions, 2);

        // Right wheel encoder
        encoder_data_t right_encoder;
        encoder_get_data(&right_encoder);
        Serial.print("  Right Pulses: ");
        Serial.println(right_encoder.pulse_count);
        Serial.print("  Right Revolutions: ");
        Serial.println(right_encoder.revolutions, 2);

        // Print sensor data
        Serial.println();
        Serial.println("Sensor Data:");

        // ToF sensors
        uint16_t left_tof = tof_get_left_distance();
        uint16_t right_tof = tof_get_right_distance();
        uint16_t top_tof = tof_get_top_distance();

        Serial.print("  Left ToF: ");
        if (left_tof == 0xFFFF) {
            Serial.println("N/A");
        } else {
            Serial.print(left_tof);
            Serial.println(" mm");
        }

        Serial.print("  Right ToF: ");
        if (right_tof == 0xFFFF) {
            Serial.println("N/A");
        } else {
            Serial.print(right_tof);
            Serial.println(" mm");
        }

        Serial.print("  Top ToF: ");
        if (top_tof == 0xFFFF) {
            Serial.println("N/A");
        } else {
            Serial.print(top_tof);
            Serial.println(" mm");
        }

        // IMU data
        float gyro_z = imu_get_gyro_z();
        float temp = imu_get_temperature();

        Serial.print("  IMU Gyro Z: ");
        Serial.print(gyro_z, 3);
        Serial.println(" rad/s");

        Serial.print("  IMU Temp: ");
        Serial.print(temp, 1);
        Serial.println(" °C");

        // Vive positioning data
        Serial.println();
        Serial.println("Vive Positioning:");

        uint16_t vive1_x = vive_get_sensor1_x();
        uint16_t vive1_y = vive_get_sensor1_y();
        bool vive1_valid = vive_sensor1_is_valid();

        Serial.print("  Sensor 1: ");
        if (vive1_valid) {
            Serial.print("X=");
            Serial.print(vive1_x);
            Serial.print(", Y=");
            Serial.println(vive1_y);
        } else {
            Serial.println("No Signal");
        }

        uint16_t vive2_x = vive_get_sensor2_x();
        uint16_t vive2_y = vive_get_sensor2_y();
        bool vive2_valid = vive_sensor2_is_valid();

        Serial.print("  Sensor 2: ");
        if (vive2_valid) {
            Serial.print("X=");
            Serial.print(vive2_x);
            Serial.print(", Y=");
            Serial.println(vive2_y);
        } else {
            Serial.println("No Signal");
        }

        Serial.println("====================================");
        Serial.println();
    }
}

/**
 * @brief 底盘控制任务
 * Chassis Control Task
 *
 * 从Web界面读取摇杆值并控制底盘运动
 * Reads joystick values from web interface and controls chassis motion
 */
static void chassis_control_task(void *pvParameters)
{
    Serial.println("Chassis control task started");

    float last_linear = 0.0f;
    float last_angular = 0.0f;

    while (1) {
        // Get chassis velocity from web interface
        float linear_velocity = web_server_get_linear_velocity();
        float angular_velocity = web_server_get_angular_velocity();

        // Only update chassis when velocity changes
        if (linear_velocity != last_linear || angular_velocity != last_angular) {
            chassis_set_velocity(linear_velocity, angular_velocity);
            last_linear = linear_velocity;
            last_angular = angular_velocity;
        }

        // 控制周期 50ms / Control period 50ms
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Arduino setup函数 - 系统初始化
 * Arduino setup function - System initialization
 *
 * 对应ESP-IDF的app_main()函数
 * Corresponds to ESP-IDF's app_main() function
 */
void setup()
{
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(2000);  // Wait for serial to stabilize

    // Force flush and print
    Serial.flush();
    Serial.println();
    Serial.println("========================================");
    Serial.println("SETUP STARTED - ESP32 Motor Control");
    Serial.println("========================================");
    Serial.flush();

    // 同时使用ESP_LOGI
    // ========== Initialize All Modules ==========
    Serial.println();
    Serial.println("Starting module initialization...");
    Serial.flush();

    // Initialize I2C bus first (for sensors)
    Serial.println("Step 1/10: Initializing I2C bus...");
    Serial.flush();
    Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);
    Wire.setClock(I2C_FREQ_HZ);
    Serial.println("OK: I2C bus initialized");
    Serial.flush();

    Serial.println("Step 2/10: Initializing motor 1 driver...");
    Serial.flush();
    esp_err_t ret = motor_driver_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Motor 1 driver init failed!");
        Serial.flush();
        while(1) { delay(1000); }
    }
    Serial.println("OK: Motor 1 driver initialized");
    Serial.flush();

    Serial.println("Step 3/10: Initializing motor 2 driver...");
    Serial.flush();
    ret = motor2_driver_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Motor 2 driver init failed!");
        Serial.flush();
        while(1) { delay(1000); }
    }
    Serial.println("OK: Motor 2 driver initialized");
    Serial.flush();

    Serial.println("Step 4/10: Initializing encoder 1...");
    Serial.flush();
    ret = encoder_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Encoder 1 init failed!");
        Serial.flush();
        while(1) { delay(1000); }
    }
    Serial.println("OK: Encoder 1 initialized");
    Serial.flush();

    Serial.println("Step 5/10: Initializing encoder 2...");
    Serial.flush();
    delay(100);
    Serial.println("  Calling encoder2_init()...");
    Serial.flush();
    ret = encoder2_init();
    Serial.println("  encoder2_init() returned");
    Serial.flush();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Encoder 2 init failed!");
        Serial.flush();
        while(1) { delay(1000); }
    }
    Serial.println("OK: Encoder 2 initialized");
    Serial.flush();

    Serial.println("Step 6/10: Initializing ToF sensors...");
    Serial.flush();
    ret = tof_init();
    if (ret != ESP_OK) {
        Serial.println("WARNING: ToF sensors init failed (continuing anyway)");
    } else {
        Serial.println("OK: ToF sensors initialized");
    }
    Serial.flush();

    Serial.println("Step 7/10: Initializing IMU sensor...");
    Serial.flush();
    ret = imu_init();
    if (ret != ESP_OK) {
        Serial.println("WARNING: IMU sensor init failed (continuing anyway)");
    } else {
        Serial.println("OK: IMU sensor initialized");
    }
    Serial.flush();

    Serial.println("Step 8/10: Initializing Vive positioning sensors...");
    Serial.flush();
    ret = vive_init();
    if (ret != ESP_OK) {
        Serial.println("WARNING: Vive sensors init failed (continuing anyway)");
    } else {
        Serial.println("OK: Vive sensors initialized");
    }
    Serial.flush();

    Serial.println("Step 9/10: Initializing chassis control...");
    Serial.flush();
    ret = chassis_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Chassis control init failed!");
        Serial.flush();
        while(1) { delay(1000); }
    }
    Serial.println("OK: Chassis control initialized");
    Serial.flush();

    Serial.println("Step 10/10: Initializing web server...");
    Serial.flush();
    ret = web_server_init();
    if (ret != ESP_OK) {
        Serial.println("ERROR: Web server init failed!");
        Serial.flush();
        while(1) { delay(1000); }
    }
    Serial.println("OK: Web server initialized");
    Serial.flush();

    Serial.println();
    Serial.println("========================================");
    Serial.println("All modules initialized successfully!");
    Serial.println("========================================");
    Serial.flush();

    // ========== Create Tasks ==========
    // ESP32-S3 双核任务分配策略：
    // Core 0: 传感器读取任务（I2C密集型）
    // Core 1: 控制任务（实时性要求高）

    Serial.println("Creating control tasks...");
    Serial.println("ESP32-S3 Dual Core Task Distribution:");
    Serial.flush();

    // ===== Core 0 任务 (传感器读取) =====

    // Create Vive positioning update task on Core 0
    // Vive需要快速响应，但不需要实时控制
    xTaskCreatePinnedToCore(vive_update_task, "vive_upd", 4096, NULL, 3, &vive_update_task_handle, 0);
    Serial.println("  [Core 0] vive_update_task (Priority 3, 50Hz)");

    // Create sensor update task (ToF + IMU) on Core 0
    // I2C传感器读取，放在Core 0避免干扰控制
    xTaskCreatePinnedToCore(sensor_update_task, "sensor_upd", 4096, NULL, 2, &sensor_update_task_handle, 0);
    Serial.println("  [Core 0] sensor_update_task (Priority 2, 20Hz)");

    // Create status monitoring task on Core 0
    // 低优先级，串口打印不影响控制
    xTaskCreatePinnedToCore(status_monitor_task, "status_mon", 4096, NULL, 1, &status_monitor_task_handle, 0);
    Serial.println("  [Core 0] status_monitor_task (Priority 1, 0.33Hz)");

    // ===== Core 1 任务 (实时控制) =====

    // Create encoder update task on Core 1
    // 编码器读取需要实时性，放在Core 1
    xTaskCreatePinnedToCore(encoder_update_task, "encoder_upd", 2048, NULL, 4, &encoder_update_task_handle, 1);
    Serial.println("  [Core 1] encoder_update_task (Priority 4, 100Hz)");

    // Create chassis control task on Core 1
    // 最高优先级，实时控制电机
    xTaskCreatePinnedToCore(chassis_control_task, "chassis_ctrl", 3072, NULL, 5, &chassis_control_task_handle, 1);
    Serial.println("  [Core 1] chassis_control_task (Priority 5, 100Hz)");

    Serial.println();
    Serial.println("========================================");
    Serial.println("System Running!");
    Serial.println("Connect to WiFi and visit web interface");
    Serial.println("(IP address shown in WiFi init above)");
    Serial.println("========================================");
    Serial.println();

    // Print system information
    Serial.println("System Information:");
    Serial.print("  CPU Frequency: ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println(" MHz");
    Serial.print("  Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    Serial.print("  Chip Model: ");
    Serial.println(ESP.getChipModel());
    Serial.print("  Chip Cores: ");
    Serial.println(ESP.getChipCores());
    Serial.println();
    Serial.flush();
}

/**
 * @brief Arduino loop函数 - 主循环
 * Arduino loop function - Main loop
 *
 * 在Arduino中，所有任务都由FreeRTOS管理，loop()可以保持空
 * In Arduino, all tasks are managed by FreeRTOS, loop() can remain empty
 */
void loop()
{
    //are you ok?
    // 所有工作都在FreeRTOS任务中完成
    // All work is done in FreeRTOS tasks
    // loop()函数可以保持空或者添加其他非关键任务
    // loop() can remain empty or add other non-critical tasks

    delay(1000);  // 避免watchdog触发 / Avoid watchdog trigger
}
