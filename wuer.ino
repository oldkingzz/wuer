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
 * - 正交编码器脉冲计数和转速测量 / Quadrature encoder pulse counting and speed
 * measurement
 * - Web界面虚拟摇杆控制 / Web interface virtual joystick control
 * - PID速度闭环控制 / PID speed closed-loop control
 * - 差速驱动运动学 / Differential drive kinematics
 */

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Adafruit_VL53L0X.h> // <--- 加这一行
#include <Arduino.h>
// 包含自定义模块头文件 / Include custom module headers
#include "src/include/chassis_v2.h" // 使用新的Chassis V2
#include "src/include/encoder.h"
#include "src/include/gpio_config.h"
#include "src/include/i2c_bus.h"
#include "src/include/imu_sensor.h"
#include "src/include/motor_driver.h"
#include "src/include/servo_control.h"
#include "src/include/tof_sensor.h"
#include "src/include/tophat.h"
#include "src/include/user_input.h"
#include "src/include/vive_navigation.h"
#include "src/include/vive_sensor.h"
#include "src/include/wall_following_v2.h"
#include <Wire.h>

// Feature Flags
#define USE_TOPHAT                                                             \
  0 // Set to 1 to enable Tophat, 0 to disable (prevent I2C conflicts when
    // missing)

static const char *TAG = "MAIN";

// 任务句柄 / Task handles
static TaskHandle_t encoder_update_task_handle = NULL;
static TaskHandle_t status_monitor_task_handle = NULL;
static TaskHandle_t chassis_control_task_handle = NULL;
static TaskHandle_t sensor_update_task_handle = NULL;
static TaskHandle_t tophat_task_handle = NULL;

// Global packet counter for Tophat
volatile uint32_t g_wifi_packet_count = 0;

/**
 * @brief 传感器更新任务（统一管理 TCA 上的 ToF + IMU 读取）
 * Sensor Update Task
 *
 * 周期性按顺序读取所有 ToF，然后读取 IMU，形成一个“完整采样周期”。
 */
static void sensor_update_task(void *pvParameters) {
  Serial.println("Sensor update task started");
  Serial.flush();

  tof_data_t top_tof, front_tof, left_front_tof, left_rear_tof;
  imu_data_t imu_data;

  // Wait a bit for sensors to stabilize
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println("Sensor task: starting sensor reads");
  Serial.flush();

  while (1) {
    // Check if Wall Following is running (Direct Mode)
    // If running, IT controls the sensors directly! We must back off.
    wf_status_t wf_stat;
    bool wf_active = (wall_following_v2_get_status(&wf_stat) == ESP_OK &&
                      wf_stat.is_running);

    if (!wf_active) {
      // 1) 读取两个实际存在的 ToF：SD1(Front) + SD2(Left-Front)
      tof_read_all(&top_tof, &front_tof, &left_front_tof, &left_rear_tof);

      // 2) 读取 IMU（时间很短），插在两次 ToF 读取之间
      esp_err_t ret = imu_read(&imu_data);
      (void)ret;
    } else {
      // Wall Following is active, just sleep to yield CPU
      // We do NOT touch the I2C bus here.
    }

    // 3) 周期 50ms 左右
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
static void encoder_update_task(void *pvParameters) {
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
static void status_monitor_task(void *pvParameters) {
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

    // Chassis V2 没有get_velocity接口，直接读取编码器
    Serial.println("Wheel Speeds:");

    // Get actual wheel speeds (raw reading)
    float left_actual_raw = encoder2_get_rpm(); // Motor 2 = Left wheel
    float right_actual_raw = encoder_get_rpm(); // Motor 1 = Right wheel

    Serial.print("  Left Actual (raw): ");
    Serial.print(left_actual_raw, 2);
    Serial.println(" RPM");
    Serial.print("  Right Actual (raw): ");
    Serial.print(right_actual_raw, 2);
    Serial.println(" RPM");

    Serial.println();
    Serial.println("Note: Chassis V2 PID debug output is in ESP_LOGI format");
    Serial.println("      Check Serial Monitor for '[CHASSIS_V2]' messages");

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

    // ToF sensors - 使用缓存接口（非阻塞）
    // 如果 WF 正在运行，这里的数据可能不再更新（因为 sensor_update_task 停了）
    // 所以应当提示 User
    wf_status_t wf_monitor;
    bool wf_running_mon =
        (wall_following_v2_get_status(&wf_monitor) == ESP_OK &&
         wf_monitor.is_running);

    if (wf_running_mon) {
      Serial.println("  ToF Sensors: [DIRECT CONTROL BY WF]");
      Serial.print("  WF Front: ");
      Serial.println(wf_monitor.tof_front_mm);
      Serial.print("  WF Right: ");
      Serial.println(wf_monitor.tof_right_mm);
    } else {
      uint16_t front_tof = tof_get_cached_front_distance();
      uint16_t left_front_tof = tof_get_cached_left_front_distance();

      Serial.print("  Front ToF (SD1): ");
      if (front_tof == 0xFFFF) {
        Serial.println("N/A");
      } else {
        Serial.print(front_tof);
        Serial.println(" mm");
      }

      Serial.print("  Left-Front ToF (SD2): ");
      if (left_front_tof == 0xFFFF) {
        Serial.println("N/A");
      } else {
        Serial.print(left_front_tof);
        Serial.println(" mm");
      }
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

    // Wall-following debug (pose + heading)
    wf_status_t wf_status;
    if (wall_following_v2_get_status(&wf_status) == ESP_OK &&
        wf_status.is_running) {
      Serial.println();
      Serial.println("Wall Following V2 Debug:");
      Serial.print("  State: ");
      Serial.println(wf_status.state);
      Serial.print("  Elapsed Time: ");
      Serial.print(wf_status.elapsed_ms);
      Serial.println(" ms");
      Serial.print("  Pose (x, y): ");
      Serial.print(wf_status.odo_x_m, 2);
      Serial.print(", ");
      Serial.print(wf_status.odo_y_m, 2);
      Serial.println(" m");
      Serial.print("  Heading: ");
      Serial.print(wf_status.odo_heading_rad * 180.0f / 3.14159f, 1);
      Serial.println(" deg");
      Serial.print("  ToF R/F: ");
      Serial.print(wf_status.tof_right_mm);
      Serial.print(" / ");
      Serial.println(wf_status.tof_front_mm);
    }

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
static void chassis_control_task(void *pvParameters) {
  Serial.println("Chassis control task started");

  float last_linear = 0.0f;
  float last_angular = 0.0f;

  while (1) {
    // Only control chassis if manual control is enabled
    // (disabled during wall following or navigation)
    // Also disabled if Tophat penalty is active
    if (tophat_is_penalized()) {
      // Stop everything!
      chassis_v2_set_velocity(0, 0);
      last_linear = 0;
      last_angular = 0;
      vTaskDelay(pdMS_TO_TICKS(100)); // Wait a bit
      continue;
    }

    if (web_server_is_manual_control_enabled()) {
      // Get chassis velocity from web interface
      float linear_velocity = web_server_get_linear_velocity();
      float angular_velocity = web_server_get_angular_velocity();

      // Only update chassis when velocity changes
      if (linear_velocity != last_linear || angular_velocity != last_angular) {
        chassis_v2_set_velocity(linear_velocity, angular_velocity);
        last_linear = linear_velocity;
        last_angular = angular_velocity;
      }
    } else {
      // Manual control disabled, reset last values
      last_linear = 0.0f;
      last_angular = 0.0f;
    }

    // 控制周期 10ms (100Hz) - 减少延迟 / Control period 10ms (100Hz) - reduce
    // latency
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief Tophat heartbeat task
 */
static void tophat_task(void *pvParameters) {
  while (1) {
    // Atomic read and reset of counter
    uint32_t count = web_server_get_packet_count_reset();

    // Try to send heartbeat
    esp_err_t ret = tophat_send_heartbeat(count);

    // If "Not inited" (-1/ESP_FAIL) is returned because not inited, try to
    // init!
    if (ret != ESP_OK) {
      // We can try to init here if it failed
      // This covers the case where Setup didn't reach it, or it failed
      // dynamically But we don't want to spam init if hardware is missing.
      // Check if it's strictly not inited state?
      // Actually tophat_init() checks g_tophat_inited internally.
      Serial.println("[TOPHAT_TASK] Retrying init...");
      tophat_init();
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/**
 * @brief Arduino setup函数 - 系统初始化

 * Arduino setup function - System initialization
 *
 * 对应ESP-IDF的app_main()函数
 * Corresponds to ESP-IDF's app_main() function
 */
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(2000); // Wait for serial to stabilize

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
  delay(100); // Wait for I2C bus to stabilize

  // Init global I2C/TCA mutex so all sensors串行访问总线
  i2c_bus_init();

  Serial.println("OK: I2C bus initialized");
  Serial.flush();

  Serial.println("Step 2/10: Initializing motor 1 driver...");
  Serial.flush();
  esp_err_t ret = motor_driver_init();
  if (ret != ESP_OK) {
    Serial.println("ERROR: Motor 1 driver init failed!");
    Serial.flush();
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK: Motor 1 driver initialized");
  Serial.flush();

  Serial.println("Step 3/10: Initializing motor 2 driver...");
  Serial.flush();
  ret = motor2_driver_init();
  if (ret != ESP_OK) {
    Serial.println("ERROR: Motor 2 driver init failed!");
    Serial.flush();
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK: Motor 2 driver initialized");
  Serial.flush();

  Serial.println("Step 4/10: Initializing encoder 1...");
  Serial.flush();
  ret = encoder_init();
  if (ret != ESP_OK) {
    Serial.println("ERROR: Encoder 1 init failed!");
    Serial.flush();
    while (1) {
      delay(1000);
    }
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
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK: Encoder 2 initialized");
  Serial.flush();

  // Step 6: IMU first, so TCA bus上还没有ToF异步任务在跑，避免冲突
  Serial.println("Step 6/10: Initializing IMU sensor...");
  Serial.flush();
  ret = imu_init();
  if (ret != ESP_OK) {
    Serial.println("WARNING: IMU sensor init failed (continuing anyway)");
  } else {
    Serial.println("OK: IMU sensor initialized");
  }
  Serial.flush();

  // Step 7: 初始化 ToF（不再启动单独的异步任务，由 sensor_update_task
  // 统一轮询）
  Serial.println("Step 7/10: Initializing ToF sensors...");
  Serial.flush();
  ret = tof_init();
  if (ret != ESP_OK) {
    Serial.println("WARNING: ToF sensors init failed (continuing anyway)");
  } else {
    Serial.println("OK: ToF sensors initialized");
  }
  Serial.flush();

  Serial.println("Step 8/10: Initializing Vive positioning sensors...");
  Serial.flush();
  ret = vive_init();
  if (ret != ESP_OK) {
    Serial.println("WARNING: Vive sensors init failed (continuing anyway)");
  } else {
    Serial.println("OK: Vive sensors initialized");

    // Start async Vive reading task (replacement for vive_update_task)
    esp_err_t vive_async_ret = vive_start_async_reading();
    if (vive_async_ret != ESP_OK) {
      Serial.println("WARNING: Failed to start Vive async reading task "
                     "(continuing anyway)");
    } else {
      Serial.println("OK: Vive async reading task started");
    }
  }
  Serial.flush();

  Serial.println("Step 9/10: Initializing chassis control V2...");
  Serial.flush();
  ret = chassis_v2_init();
  if (ret != ESP_OK) {
    Serial.println("ERROR: Chassis V2 control init failed!");
    Serial.flush();
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK: Chassis V2 control initialized");
  Serial.flush();

  Serial.println("Step 9.5/10: Initializing servo...");
  Serial.flush();
  ret = servo_init();
  if (ret != ESP_OK) {
    Serial.println("WARNING: Servo init failed");
  } else {
    Serial.println("OK: Servo initialized");
  }
  Serial.flush();

  // 在WiFi初始化前检查内存
  Serial.println();
  Serial.println("========================================");
  Serial.println("Memory Status BEFORE WiFi Init:");
  Serial.print("  Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("  Largest Free Block: ");
  Serial.print(ESP.getMaxAllocHeap());
  Serial.println(" bytes");
  Serial.println("========================================");
  Serial.flush();

  Serial.println("Step 10/11: Initializing web server...");
  Serial.flush();
  ret = web_server_init();
  if (ret != ESP_OK) {
    Serial.println("ERROR: Web server init failed!");
    Serial.flush();
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK: Web server initialized");
  Serial.flush();

  // Initialize Tophat (Conditional)
  if (USE_TOPHAT) {
    tophat_init();
  } else {
    Serial.println("Tophat disabled by USE_TOPHAT flag.");
  }

  // 在导航初始化前检查内存
  Serial.println();
  Serial.println("========================================");
  Serial.println("Memory Status BEFORE Navigation Init:");
  Serial.print("  Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("  Largest Free Block: ");
  Serial.print(ESP.getMaxAllocHeap());
  Serial.println(" bytes");
  Serial.println("========================================");
  Serial.flush();

  Serial.println("Step 11/12: Initializing navigation system...");
  Serial.flush();

  ret = vive_nav_init();
  if (ret != ESP_OK) {
    Serial.println("ERROR: Navigation init failed!");
    Serial.flush();
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK: Navigation system initialized");
  Serial.flush();

  // 导航初始化后检查内存
  Serial.println();
  Serial.println("========================================");
  Serial.println("Memory Status AFTER Navigation Init:");
  Serial.print("  Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("  Largest Free Block: ");
  Serial.print(ESP.getMaxAllocHeap());
  Serial.println(" bytes");
  Serial.println("========================================");
  Serial.flush();

  Serial.println("Step 12/12: Initializing wall following V2 system...");
  Serial.flush();
  ret = wall_following_v2_init();
  if (ret != ESP_OK) {
    Serial.println(
        "WARNING: Wall following V2 init failed (continuing anyway)");
  } else {
    Serial.println("OK: Wall following V2 system initialized");
  }
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

  // Vive 异步读取任务现在在 vive_sensor 模块内部创建，这里只做说明
  Serial.println(
      "  [Core 0] Vive async reading is handled inside vive_sensor module");

  // Create sensor update task (ToF + IMU) on Core 0
  // I2C传感器读取，放在Core 0避免干扰控制
  // 提高优先级至10，避免被其他低优先级的WiFi辅助任务饿死
  xTaskCreatePinnedToCore(sensor_update_task, "sensor_upd", 3072, NULL, 10,
                          &sensor_update_task_handle, 0);
  Serial.println(
      "  [Core 0] sensor_update_task (Priority 10, 20Hz, Stack: 3KB)");

  // Create status monitoring task on Core 0
  // 低优先级，串口打印不影响控制
  xTaskCreatePinnedToCore(status_monitor_task, "status_mon", 2048, NULL, 1,
                          &status_monitor_task_handle, 0);
  Serial.println(
      "  [Core 0] status_monitor_task (Priority 1, 0.33Hz, Stack: 2KB)");

  // ===== Core 1 任务 (实时控制) =====

  // Create encoder update task on Core 1
  // 编码器读取需要实时性，放在Core 1
  xTaskCreatePinnedToCore(encoder_update_task, "encoder_upd", 2048, NULL, 4,
                          &encoder_update_task_handle, 1);
  Serial.println(
      "  [Core 1] encoder_update_task (Priority 4, 100Hz, Stack: 2KB)");

  // Create chassis control task on Core 1
  // 最高优先级，实时控制电机
  xTaskCreatePinnedToCore(chassis_control_task, "chassis_ctrl", 2560, NULL, 5,
                          &chassis_control_task_handle, 1);
  Serial.println(
      "  [Core 1] chassis_control_task (Priority 5, 100Hz, Stack: 2.5KB)");

  if (USE_TOPHAT) {
    xTaskCreatePinnedToCore(tophat_task, "tophat", 2048, NULL, 3,
                            &tophat_task_handle, 1);
    Serial.println("  [Core 1] tophat_task (Priority 3, 2Hz, Stack: 2KB)");
  } else {
    Serial.println("  [Core 1] tophat_task DISABLED");
  }

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
// 声明测试用
#include "src/include/astar.h"
#include "src/include/grid_map.h"

void loop() {
  // Serial Debug Console
  if (Serial.available() > 0) {
    char c = Serial.read();

    // Command 'm': Check Map Cost at current robot pose
    if (c == 'm') {
      vive_pose_t pose;
      vive_nav_get_pose(&pose);
      uint8_t cost = grid_map_get_cost(pose.x, pose.y);
      Serial.printf("DEBUG MAP: Pose(%d, %d) Cost=%d (Obstacle if >= 255)\n",
                    pose.x, pose.y, cost);
    }

    // Command 't': Test A* Plan from current pose to (40, 100)
    if (c == 't') {
      vive_pose_t pose;
      vive_nav_get_pose(&pose);
      Serial.printf("DEBUG PLAN: Planning from (%d, %d) to (40, 100)...\n",
                    pose.x, pose.y);

      path_t test_path;
      esp_err_t ret = astar_plan_path(pose.x, pose.y, 40, 100, &test_path);

      if (ret == ESP_OK) {
        Serial.printf("DEBUG PLAN: Success! Length=%d\n", test_path.length);
      } else {
        Serial.printf("DEBUG PLAN: Failed! Err=%d\n", ret);
        // Diagnose why
        if (!grid_map_is_free(pose.x, pose.y))
          Serial.println("  -> Start Blocked");
        if (!grid_map_is_free(40, 100))
          Serial.println("  -> Goal Blocked");
      }
    }
  }

  delay(100);
}
