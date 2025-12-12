/**
 * @file wall_following_v2.cpp
 * @brief Wall Following V2 - Sequential State Machine Implementation
 */

#include "include/wall_following_v2.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/chassis_v2.h"
#include "include/encoder.h"
#include "include/imu_sensor.h"
#include "include/tof_sensor.h"
#include <Arduino.h>
#include <cmath>

static const char *TAG = "WF_V2";
static bool g_inited = false;
static TaskHandle_t g_wf_task_handle = NULL;

// Global Status
static wf2_status_t g_status = {.state = WF2_IDLE,
                                .stage = WF2_STAGE_CONVEX_CORNER_RT,
                                .current_x = 0,
                                .current_y = 0,
                                .current_heading = 0,
                                .tof_front = 0,
                                .tof_right = 0,
                                .is_running = false};

// ==================== Stage Handler Prototypes ====================
// Return true if stage is finished and should switch to next stage
static bool process_convex_corner_rt(void);
static bool process_trajectory_top(void);
static bool process_convex_corner_lt(void);
static bool process_long_edge_left(void);
static bool process_convex_corner_lb(void);
static bool process_trajectory_bottom(void);
static bool process_convex_corner_rb(void);
static bool process_long_edge_right(void);

// ==================== State Machine Main Loop ====================

static void wf2_task(void *arg) {
  ESP_LOGI(TAG, "Wall Following Task Started");
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(50); // 20Hz loop

  while (1) {
    // Only run logic if system is in RUNNING state
    if (g_status.state == WF2_RUNNING) {

      // Update Sensor Data (Optional, for reference)
      g_status.tof_front = tof_get_cached_front_distance();
      g_status.tof_right = tof_get_cached_left_front_distance();
      g_status.current_heading +=
          imu_get_gyro_z() *
          0.05f; // Simple Integration, better use dedicated Odom

      bool stage_done = false;

      // Execute current stage logic
      switch (g_status.stage) {
      case WF2_STAGE_CONVEX_CORNER_RT:
        stage_done = process_convex_corner_rt();
        if (stage_done) {
          ESP_LOGI(TAG, "Finished CONVEX_CORNER_RT -> Entering TRAJECTORY_TOP");
          g_status.stage = WF2_STAGE_TRAJECTORY_TOP;
        }
        break;

      case WF2_STAGE_TRAJECTORY_TOP:
        stage_done = process_trajectory_top();
        if (stage_done) {
          ESP_LOGI(TAG, "Finished TRAJECTORY_TOP -> Entering CONVEX_CORNER_LT");
          g_status.stage = WF2_STAGE_CONVEX_CORNER_LT;
        }
        break;

      case WF2_STAGE_CONVEX_CORNER_LT:
        stage_done = process_convex_corner_lt();
        if (stage_done) {
          ESP_LOGI(TAG, "Finished CONVEX_CORNER_LT -> Entering LONG_EDGE_LEFT");
          g_status.stage = WF2_STAGE_LONG_EDGE_LEFT;
        }
        break;

      case WF2_STAGE_LONG_EDGE_LEFT:
        stage_done = process_long_edge_left();
        if (stage_done) {
          ESP_LOGI(TAG, "Finished LONG_EDGE_LEFT -> Entering CONVEX_CORNER_LB");
          g_status.stage = WF2_STAGE_CONVEX_CORNER_LB;
        }
        break;

      case WF2_STAGE_CONVEX_CORNER_LB:
        stage_done = process_convex_corner_lb();
        if (stage_done) {
          ESP_LOGI(TAG,
                   "Finished CONVEX_CORNER_LB -> Entering TRAJECTORY_BOTTOM");
          g_status.stage = WF2_STAGE_TRAJECTORY_BOTTOM;
        }
        break;

      case WF2_STAGE_TRAJECTORY_BOTTOM:
        stage_done = process_trajectory_bottom();
        if (stage_done) {
          ESP_LOGI(TAG,
                   "Finished TRAJECTORY_BOTTOM -> Entering CONVEX_CORNER_RB");
          g_status.stage = WF2_STAGE_CONVEX_CORNER_RB;
        }
        break;

      case WF2_STAGE_CONVEX_CORNER_RB:
        stage_done = process_convex_corner_rb();
        if (stage_done) {
          ESP_LOGI(TAG,
                   "Finished CONVEX_CORNER_RB -> Entering LONG_EDGE_RIGHT");
          g_status.stage = WF2_STAGE_LONG_EDGE_RIGHT;
        }
        break;

      case WF2_STAGE_LONG_EDGE_RIGHT:
        stage_done = process_long_edge_right();
        if (stage_done) {
          ESP_LOGI(TAG, "Finished LONG_EDGE_RIGHT -> ALL DONE. Stopping.");
          g_status.stage = WF2_STAGE_DONE;

          // Auto-Stop the system
          wall_following_v2_stop();
        }
        break;

      case WF2_STAGE_DONE:
      default:
        chassis_v2_stop();
        break;
      }
    } else {
      // Idle state, ensure motors are stopped (or handled by stop())
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelayUntil(&last_wake, period);
  }
}

// ==================== Helper Functions ====================

// Static variables for helper state
static int32_t help_start_L = 0;
static int32_t help_start_R = 0;
static bool help_started = false;

// Generic Turn Function
// Generic Turn Function
static bool move_turn_relative(float target_angle_deg) {
  const float wheel_dia = 0.065f;
  const float wheel_base = 0.15f;
  const float cpr = 3200.0f;

  // Slower turn speed for 180 to prevent slip
  float turn_speed = 1.0f;
  if (fabs(target_angle_deg) > 90.0f)
    turn_speed = 0.8f;

  if (!help_started) {
    help_start_L = encoder2_get_count();
    help_start_R = encoder_get_count();
    help_started = true;

    // Sign determines direction: + = CCW (Left), - = CW (Right)
    float omega = (target_angle_deg > 0) ? turn_speed : -turn_speed;
    chassis_v2_set_velocity(0.0f, omega);
    return false;
  }

  int32_t curr_L = encoder2_get_count();
  int32_t curr_R = encoder_get_count();
  float delta_L = -(float)(curr_L - help_start_L);
  float delta_R = (float)(curr_R - help_start_R); // Forward = +

  float dist_L = (delta_L / cpr) * (3.14159f * wheel_dia);
  float dist_R = (delta_R / cpr) * (3.14159f * wheel_dia);
  float angle_rad = (dist_R - dist_L) / wheel_base;
  float current_deg = angle_rad * (180.0f / 3.14159f);

  if (fabs(current_deg) >= fabs(target_angle_deg)) {
    chassis_v2_stop();
    help_started = false;
    return true;
  }
  return false;
}

static bool move_turn_ccw_90(void) {
  return move_turn_relative(80.0f); // Keep 80 deg as requested
}

static bool move_dist_straight(float target_m) {
  if (!help_started) {
    help_start_L = encoder2_get_count();
    help_start_R = encoder_get_count();
    help_started = true;
    chassis_v2_set_velocity(0.1f, 0.0f); // 0.1 m/s
    return false;
  }

  int32_t curr_L = encoder2_get_count();
  int32_t curr_R = encoder_get_count();
  float delta_L = -(float)(curr_L - help_start_L);
  float delta_R = (float)(curr_R - help_start_R);

  const float cpr = 3200.0f;
  const float wheel_dia = 0.065f;
  float dL = (delta_L / cpr) * 3.14159f * wheel_dia;
  float dR = (delta_R / cpr) * 3.14159f * wheel_dia;
  float avg = (dL + dR) / 2.0f;

  if (avg >= target_m) {
    chassis_v2_stop();
    help_started = false;
    return true;
  }
  return false;
}

// ==================== Stage Implementations ====================
// Fill in your logic inside these functions.
// Return 'true' to break out and move to the next stage.

/**
 * @brief Drive straight at specified velocity
 * Simple wrapper for chassis control.
 */
static void move_at_velocity(float linear_m_s) {
  chassis_v2_set_velocity(linear_m_s, 0.0f);
}

// 1. 凸角_RT (Convex Corner RT)
// Logic: Drive Straight -> Stop if Front < 8" -> Turn 90 CCW -> Check Front ->
// (Turn 90 CCW again if blocked)
static bool process_convex_corner_rt(void) {
  static int step = 0;
  const float THRESHOLD_MM = 130.0f; // 8 inches ~ 203mm

  switch (step) {
  case 0:                   // State 1: Drive Straight
    move_at_velocity(0.1f); // Normal speed

    // Check Trigger: Front ToF < 8 inches
    // Note: Use Front ToF as primary obstacle trigger
    {
      uint16_t front_dist = tof_get_cached_front_distance();
      if (front_dist != 0xFFFF && front_dist < THRESHOLD_MM) {
        chassis_v2_stop();
        step = 1;
      }
    }
    break;

  case 1: // Action: Turn 90 CCW
    if (move_turn_ccw_90()) {
      step = 2; // Check next
    }
    break;

  case 2: // Align Distance before second turn
  {
    // Target Range: 8cm - 12cm (80mm - 120mm)
    const float DIST_MIN_MM = 70.0f;
    const float DIST_MAX_MM = 100.0f;

    uint16_t front = tof_get_cached_front_distance();
    if (front == 0xFFFF) {
      chassis_v2_stop(); // Stop if sensor data is invalid
      return false;
    }

    if (front < DIST_MIN_MM) {
      // Too close (< 8cm) -> Move Backward
      move_at_velocity(-0.1f); // Slow reverse
    } else if (front > DIST_MAX_MM) {
      // Too far (> 12cm) -> Move Forward
      move_at_velocity(0.1f); // Slow forward
    } else {
      // In Range [8cm, 12cm] -> Ready to Turn
      chassis_v2_stop();
      step = 99; // Insert Wait State to ensure full stop
    }
  } break;

  case 99: // Wait for robot to stabilize
  {
    static int wait_tick = 0;
    if (++wait_tick > 10) { // 10 * 50ms = 500ms
      wait_tick = 0;
      step = 3;
    }
  } break;

  case 3: // Additional Action: Turn 90 CCW again
    if (move_turn_ccw_90()) {
      step = 4;
    }
    break;

  case 4:        // Done
    step = 0;    // Reset for future calls
    return true; // Exit Stage

  default:
    step = 0;
    break;
  }

  return false; // Continue in this stage
}

// Helper: Move in an Arc
// radius_m: Turn radius in meters
// angle_deg: Target arc angle (Positive = Left/CCW, Negative = Right/CW)
// linear_speed: Forward speed m/s
static bool move_arc(float radius_m, float angle_deg, float linear_speed) {
  if (!help_started) {
    help_start_L = encoder2_get_count();
    help_start_R = encoder_get_count();
    help_started = true;

    // Calculate Omega (w) = v / r
    // If angle is negative (CW), angular velocity should be negative
    float w = linear_speed / radius_m;
    if (angle_deg < 0)
      w = -w;

    chassis_v2_set_velocity(linear_speed, w);
    return false;
  }

  // Monitor Angle Integration
  int32_t curr_L = encoder2_get_count();
  int32_t curr_R = encoder_get_count();
  float delta_L = -(float)(curr_L - help_start_L);
  float delta_R = (float)(curr_R - help_start_R);

  // Wheel params
  const float wheel_dia = 0.065f;
  const float wheel_base = 0.15f;
  const float cpr = 3200.0f;

  float dist_L = (delta_L / cpr) * (3.14159f * wheel_dia);
  float dist_R = (delta_R / cpr) * (3.14159f * wheel_dia);

  // Current Rotation
  float angle_rad = (dist_R - dist_L) / wheel_base;
  float current_deg = angle_rad * (180.0f / 3.14159f);

  if (fabs(current_deg) >= fabs(angle_deg)) {
    chassis_v2_stop();
    help_started = false;
    return true;
  }
  return false;
}

// 2. 轨迹_T (Trajectory Top)
// Logic: Drive 90% of a Semicircle Arc (User Request)
// Exit early if Front ToF < 7cm
static bool process_trajectory_top(void) {
  // 13 inches = 0.3302 meters
  const float RADIUS_M = 0.27f;
  const float LINEAR_V = 0.1f;
  // Arc Angle: 90% of 180 = 162 degrees
  // Direction: CW (Right) negative

  if (move_arc(RADIUS_M, -162.0f, LINEAR_V)) {
    return true;
  }
  return false;
}

// 3. 凸角_LT (Convex Corner LT)
// Logic: Turn 90 CCW -> Wall Follow (Right side, 50mm target) until Front <
// 13cm -> Stop -> Turn 90 CCW
static bool process_convex_corner_lt(void) {
  static int step = 0;
  const float FRONT_STOP_MM = 130.0f; // 13cm
  const float BASE_SPEED = 0.05f;

  switch (step) {
  case 0: // Action: Turn 90 CCW (Align with wall)
    if (move_turn_ccw_90()) {
      step = 99; // 进入等待状态
    }
    break;

  case 99: // Wait 2s after turn to stabilize
  {
    static int wait_tick = 0;
    if (++wait_tick > 40) { // 40 * 50ms = 2000ms = 2s
      wait_tick = 0;
      step = 1; // 进入下一阶段
    }
  } break;

  case 1: // Action: Drive Straight until Front < 13cm
  {
    // 1. 每次进入 Case 都获取最新的传感器数据
    uint16_t front = tof_get_cached_front_distance();

    // 2. 判断逻辑
    if (front == 0xFFFF) {
      // 传感器无效，继续前进（不改变状态）
      chassis_v2_set_velocity(BASE_SPEED, 0.0f);
    } else if (front > FRONT_STOP_MM) {
      // 距离还不够近，继续保持前进速度
      chassis_v2_set_velocity(BASE_SPEED, 0.0f);
    } else {
      // 距离达标（front <= FRONT_STOP_MM 且有效），停车并切换状态
      chassis_v2_stop();
      step = 2; // 切换到下一个状态
    }
  } break;

  case 2: // Action: Turn 90 CCW (Final)
    if (move_turn_ccw_90()) {
      step = 3;
    }
    break;

  case 3: // Done
    step = 0;
    return true;

  default:
    step = 0;
    break;
  }

  return false;
}

// 4. 长边_L (Long Edge Left)
// Logic: Wall Follow Right (Target 70mm) ALWAYS.
// Condition: Drive 0.5m blind -> Then check Front ToF < 10cm to finish.
static bool process_long_edge_left(void) {
  // State variables
  static bool stage_started = false;
  static int32_t start_enc_L = 0;
  static int32_t start_enc_R = 0;

  // Constants
  const float WALL_TARGET_MM = 80.0f; // 7cm
  const float FRONT_STOP_MM = 120.0f; // 10cm
  const float BLIND_DIST_M = 0.5f;    // 0.5m blind zone
  const float KP_WALL = 0.02f;
  const float BASE_SPEED = 0.2f;

  if (!stage_started) {
    start_enc_L = encoder2_get_count();
    start_enc_R = encoder_get_count();
    stage_started = true;
  }

  // 1. Calculate Distance Traveled (Odom)
  int32_t curr_L = encoder2_get_count();
  int32_t curr_R = encoder_get_count();

  // Invert Left because of hardware reversal
  float delta_L = -(float)(curr_L - start_enc_L);
  float delta_R = (float)(curr_R - start_enc_R);

  const float cpr = 3200.0f;
  const float wheel_dia = 0.065f;

  float dist_L = (delta_L / cpr) * (3.14159f * wheel_dia);
  float dist_R = (delta_R / cpr) * (3.14159f * wheel_dia);
  float avg_dist_m = (dist_L + dist_R) / 2.0f;

  // 2. Wall Following Logic (Continuous)
  uint16_t right_dist = tof_get_cached_left_front_distance();
  float angular_correction = 0.0f;

  if (right_dist != 0xFFFF && right_dist < 600) { // Valid range
    float error = (float)right_dist - WALL_TARGET_MM;
    angular_correction =
        -KP_WALL * error; // Right Wall: Too far (>Target) -> Turn Right (-)

    // Clamp
    if (angular_correction > 1.0f)
      angular_correction = 1.0f;
    if (angular_correction < -1.0f)
      angular_correction = -1.0f;
  }

  // 3. Check Exit Condition
  bool stop_condition = false;
  if (avg_dist_m > BLIND_DIST_M) {
    // Only check sensor after 0.5m
    uint16_t front = tof_get_cached_front_distance();
    if (front != 0xFFFF && front < FRONT_STOP_MM) {
      stop_condition = true;
    }
  }

  if (stop_condition) {
    // Stop first
    chassis_v2_stop();

    // Final Action: Turn 90 CCW before exiting
    if (move_turn_ccw_90()) {
      stage_started = false; // Reset
      return true;           // Done, enter next stage
    }
  } else {
    chassis_v2_set_velocity(BASE_SPEED, angular_correction);
  }

  return false;
}

// 5. 凸角_LB (Convex Corner LB)
// Logic: Drive Forward -> Stop if Front < 8cm -> Turn 90 CCW
static bool process_convex_corner_lb(void) {
  static int step = 0;
  const float FRONT_THRESHOLD_MM = 80.0f; // 8cm

  switch (step) {
  case 0: // Drive Straight
    move_at_velocity(0.1f);

    // Check Front Sensor
    {
      uint16_t front = tof_get_cached_front_distance();
      if (front != 0xFFFF && front < FRONT_THRESHOLD_MM) {
        chassis_v2_stop();
        step = 1;
      }
    }
    break;

  case 1: // Turn 90 CCW
    if (move_turn_ccw_90()) {
      step = 2; // Done
    }
    break;

  case 2:
    step = 0;
    return true;

  default:
    step = 0;
    break;
  }

  return false;
}

// 6. 轨迹_B (Trajectory Bottom)
// Logic: Drive 90% of a Semicircle Arc
// Exit early if Front ToF < 7cm
static bool process_trajectory_bottom(void) {
  const float RADIUS_M = 0.26f;
  const float LINEAR_V = 0.1f;
  const float ARC_ANGLE = -162.0f;   // 90% of 180 CW
  const float FRONT_EXIT_MM = 70.0f; // 7cm

  // Check Front Obstacle for early exit
  uint16_t front = tof_get_cached_front_distance();
  if (front != 0xFFFF && front < FRONT_EXIT_MM) {
    chassis_v2_stop();
    help_started = false; // Reset helper state
    return true;
  }

  if (move_arc(RADIUS_M, ARC_ANGLE, LINEAR_V)) {
    return true;
  }
  return false;
}

// 7. 凸角_RB (Convex Corner RB)
// Logic: Wall Follow Right (Target 50mm) -> Stop if Front < 13cm -> Turn 90
// CCW
static bool process_convex_corner_rb(void) {
  static int step = 0;
  const float WALL_TARGET_MM = 50.0f;
  const float FRONT_STOP_MM = 130.0f; // 13cm
  const float KP_WALL = 0.02f;
  const float BASE_SPEED = 0.1f;

  switch (step) {
  case 0: // Wall Following with Front Check
  {
    // Check Front
    uint16_t front = tof_get_cached_front_distance();
    if (front != 0xFFFF && front < FRONT_STOP_MM) {
      chassis_v2_stop();
      step = 1;
      break;
    }

    // Wall Follow Logic
    uint16_t right_dist = tof_get_cached_left_front_distance();
    float angular_correction = 0.0f;

    if (right_dist != 0xFFFF && right_dist < 400) {
      float error = (float)right_dist - WALL_TARGET_MM;
      angular_correction = -KP_WALL * error;

      if (angular_correction > 1.0f)
        angular_correction = 1.0f;
      if (angular_correction < -1.0f)
        angular_correction = -1.0f;
    }

    chassis_v2_set_velocity(BASE_SPEED, angular_correction);
  } break;

  case 1: // Turn 90 CCW
    if (move_turn_ccw_90()) {
      step = 2;
    }
    break;

  case 2: // Done
    step = 0;
    return true;

  default:
    step = 0;
    break;
  }

  return false;
}

// 8. 场边_R (Long Edge Right)
// Logic: Wall Follow Right (Target 50mm) -> Stop if Front < 8cm -> FINISH
static bool process_long_edge_right(void) {
  const float WALL_TARGET_MM = 50.0f;
  const float FRONT_STOP_MM = 80.0f; // 8cm
  const float KP_WALL = 0.02f;
  const float BASE_SPEED = 0.1f;

  // Check Front Obstacle (End of Course)
  uint16_t front = tof_get_cached_front_distance();
  if (front != 0xFFFF && front < FRONT_STOP_MM) {
    chassis_v2_stop();
    return true; // MISSION COMPLETE
  }

  // Wall Follow Logic
  uint16_t right_dist = tof_get_cached_left_front_distance();
  float angular_correction = 0.0f;

  if (right_dist != 0xFFFF && right_dist < 400) {
    float error = (float)right_dist - WALL_TARGET_MM;
    angular_correction = -KP_WALL * error;

    if (angular_correction > 1.0f)
      angular_correction = 1.0f;
    if (angular_correction < -1.0f)
      angular_correction = -1.0f;
  }

  chassis_v2_set_velocity(BASE_SPEED, angular_correction);

  return false;
}

// ==================== System Interface ====================

esp_err_t wall_following_v2_init(void) {
  if (g_inited)
    return ESP_OK;

  // Create the task
  xTaskCreatePinnedToCore(wf2_task, "wf2_task", 4096, NULL, 5,
                          &g_wf_task_handle, 1);

  g_inited = true;
  ESP_LOGI(TAG, "Wall Following V2 Initialized");
  return ESP_OK;
}

esp_err_t wall_following_v2_start(void) {
  g_status.state = WF2_RUNNING;
  g_status.stage = WF2_STAGE_CONVEX_CORNER_RT; // Always reset to start? Or
                                               // resume? Let's reset.
  g_status.is_running = true;
  ESP_LOGI(TAG, "Wall Following STARTED");
  return ESP_OK;
}

esp_err_t wall_following_v2_stop(void) {
  g_status.state = WF2_STOPPED;
  g_status.is_running = false;
  chassis_v2_stop();
  ESP_LOGI(TAG, "Wall Following STOPPED");
  return ESP_OK;
}

esp_err_t wall_following_v2_get_status(wf2_status_t *status) {
  if (!status)
    return ESP_FAIL;
  *status = g_status;
  return ESP_OK;
}
