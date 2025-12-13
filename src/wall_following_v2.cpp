/**
 * @file wall_following_v2.cpp
 * @brief Wall Following Interface (Simplified PID + Reactive Avoidance)
 */

#include "include/wall_following_v2.h"
#include "Arduino.h"
#include "esp_log.h"
#include "include/chassis_v2.h"
#include "include/tof_sensor.h"
#include <math.h>

static const char *TAG = "WF_V2";

static wf_status_t g_status = {
    .state = WF_STATE_IDLE,
    .tof_front_mm = 0,
    .tof_right_mm = 0,
    .odo_x_m = 0.0f,
    .odo_y_m = 0.0f,
    .odo_heading_rad = 0.0f,
    .elapsed_ms = 0,
    .is_running = false,
};

// PID Params
static float Kp = 0.006f;
static float Kd = 0.003f;

esp_err_t wall_following_v2_init(void) { return ESP_OK; }

esp_err_t wall_following_v2_run_blocking(void) {
  ESP_LOGI(TAG, "Starting Wall Following V2 (Simple PID)...");
  g_status.is_running = true;
  g_status.state = WF_STATE_FOLLOW_WALL;

  uint32_t start_time = millis();
  uint32_t last_loop_time = start_time;
  float last_error = 0.0f;

  while (g_status.is_running) {
    uint32_t now = millis();
    uint32_t dt_ms = now - last_loop_time;

    // Enforce 20ms loop rate (50Hz)
    if (dt_ms < WF_LOOP_PERIOD_MS) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    last_loop_time = now;
    g_status.elapsed_ms = now - start_time;

    // 1. Read Sensors
    tof_data_t front_data, right_data;
    tof_read(TOF_FRONT, &front_data);
    tof_read(TOF_LEFT_FRONT, &right_data); // Alias for Right Side

    if (front_data.valid)
      g_status.tof_front_mm = front_data.distance_mm;
    else
      g_status.tof_front_mm = 0xFFFF;

    if (right_data.valid)
      g_status.tof_right_mm = right_data.distance_mm;
    else
      g_status.tof_right_mm = 0xFFFF;

    // 2. Control Logic
    float linear_cmd = WF_BASE_SPEED; // Default forward
    float angular_cmd = 0.0f;

    // A. Front Critical (Backup)
    // Threshold: WF_FRONT_BACKUP_MM (130mm)
    if (front_data.valid && front_data.distance_mm < WF_FRONT_BACKUP_MM) {
      g_status.state = WF_STATE_RAMP; // Reuse RAMP state for BACKUP logic
      // Backup Logic
      linear_cmd = -WF_BACKUP_SPEED; // Move backward
      angular_cmd = 0.0f;            // Straight back
      // Reset PID error so we don't jump when resuming
      last_error = 0.0f;
    }
    // B. Right Emergency (Rebound/Backup)
    // Threshold: WF_RIGHT_EMERGENCY_MM (80mm)
    else if (right_data.valid &&
             right_data.distance_mm < WF_RIGHT_EMERGENCY_MM) {
      g_status.state =
          WF_STATE_RAMP; // Using RAMP state for "Safety/Recovery" actions
      // Rebound Logic: Backup away from the wall
      linear_cmd = -WF_BACKUP_SPEED;
      angular_cmd = 0.0f;
      last_error = 0.0f;
    }
    // C. Front Warning (Turn Left)
    // Threshold: WF_FRONT_STOP_MM (200mm)
    else if (front_data.valid && front_data.distance_mm < WF_FRONT_STOP_MM) {
      g_status.state = WF_STATE_FRONT_BLOCKED;
      // Turn Left Logic
      linear_cmd = 0.0f;
      angular_cmd = 1.2f; // Positive = Left Turn
      last_error = 0.0f;
    }
    // C. Normal Wall Following (PID)
    else {
      g_status.state = WF_STATE_FOLLOW_WALL;

      float current_right_mm =
          (right_data.valid) ? (float)right_data.distance_mm : 2000.0f;
      if (current_right_mm > 1500)
        current_right_mm = 1500;

      // Error = Target - Measured
      // If Measured (500) > Target (230) -> Error = -270 -> Turn Right
      // (Negative) If Measured (100) < Target (230) -> Error = +130 -> Turn
      // Left (Positive)
      float error = (float)WF_TARGET_WALL_DIST_MM - current_right_mm;

      // Derivative
      float valid_dt = (float)dt_ms / 1000.0f;
      if (valid_dt <= 0.001f)
        valid_dt = 0.02f;
      float d_error = (error - last_error) / valid_dt;
      last_error = error;

      // PID
      angular_cmd = Kp * error + Kd * d_error;

      // Clamp Angular
      if (angular_cmd > WF_MAX_ANGULAR)
        angular_cmd = WF_MAX_ANGULAR;
      if (angular_cmd < -WF_MAX_ANGULAR)
        angular_cmd = -WF_MAX_ANGULAR;
    }

    // 3. Drive
    chassis_v2_set_velocity(linear_cmd, angular_cmd);

    // Logging
    if (now % 500 < 20) {
      ESP_LOGI(TAG, "St:%d F:%d R:%d | Lin:%.2f Ang:%.2f", g_status.state,
               g_status.tof_front_mm, g_status.tof_right_mm, linear_cmd,
               angular_cmd);
    }
  }

  chassis_v2_stop();
  ESP_LOGI(TAG, "Wall Following Stopped");
  return ESP_OK;
}

void wall_following_v2_request_stop(void) { g_status.is_running = false; }

esp_err_t wall_following_v2_get_status(wf_status_t *status) {
  if (status) {
    *status = g_status;
  }
  return ESP_OK;
}

esp_err_t wall_following_v2_start(void) { return ESP_OK; }

esp_err_t wall_following_v2_stop(void) {
  g_status.is_running = false;
  return ESP_OK;
}
