/**
 * @file wall_following_v2.cpp
 * @brief Wall Following V2 - Reactive Implementation
 */

#include "include/wall_following_v2.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/chassis_v2.h"
#include "include/tof_sensor.h"
#include "include/vive_navigation.h" // Added to stop Nav when WF starts
#include <math.h>

static const char *TAG = "WF_V2";
static bool g_inited = false;
static TaskHandle_t g_wf_task_handle = NULL;

// Tunings
#define WF_TARGET_DIST_MM 130 // Target distance from right wall
#define WF_FRONT_STOP_MM                                                       \
  350 // Trigger left turn if front wall is closer than this
#define WF_MAX_VALID_MM 600 // Max valid range to consider as "Wall"
#define WF_KP                                                                  \
  0.008f // P Gain for wall following (Increased to improve responsiveness)
#define WF_BASE_SPEED 0.15f // Base linear speed m/s
#define WF_TURN_SPEED                                                          \
  1.2f // Left Turn Speed (Inner Corner) - BOOSTED for weak Right Motor
#define WF_MAX_TURN_CMD 1.5f // Max angular limit

// Global Status
static wf2_status_t g_status = {.state = WF2_IDLE,
                                .stage = WF2_STAGE_DONE,
                                .current_x = 0,
                                .current_y = 0,
                                .current_heading = 0,
                                .tof_front = 0,
                                .tof_right = 0,
                                .is_running = false};

static void wf2_task(void *arg) {
  ESP_LOGI(TAG, "Wall Following Task Started");
  const TickType_t period = pdMS_TO_TICKS(50); // 20Hz loop

  while (1) {
    // 1. Check if Running
    if (!g_status.is_running) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // 2. Read Sensors (Front=SD1, Right=SD2)
    // Use cached data from sensor_update_task in wuer.ino
    // Note: SD2 (physically Right) is mapped to Left-Front channel
    uint16_t dist_front = tof_get_cached_front_distance();
    uint16_t dist_right = tof_get_cached_left_front_distance();

    // Handle invalid readings (0xFFFF)
    if (dist_front == 0xFFFF)
      dist_front = 0;
    if (dist_right == 0xFFFF)
      dist_right = 0;

    g_status.tof_front = dist_front;
    g_status.tof_right = dist_right;

    float lin_cmd = 0.0f;
    float ang_cmd = 0.0f;

    // 3. Logic: Right Wall Following
    // Priority 1: Front Obstacle (Inner Corner) -> Turn Left
    if (dist_front > 10 && dist_front < WF_FRONT_STOP_MM) {

      // Safety: If TOO CLOSE (considering sensor offset), Back up!
      if (dist_front < 130) {
        lin_cmd = -0.15f; // Back up slowly
        ang_cmd = 0.0f;
        // ESP_LOGD(TAG, "Too Close! Backing up: Front=%d", dist_front);
      } else {
        // Normal Wall Ahead! Stop and Turn Left (In Place)
        lin_cmd = 0.0f;
        ang_cmd = WF_TURN_SPEED; // +1.2 rad/s (Left)
        // ESP_LOGD(TAG, "Corner Left: Front=%d", dist_front);
      }
    } else {
      // Priority 2: Follow Right Wall
      if (dist_right > 10 && dist_right < WF_MAX_VALID_MM) {
        // PID Control
        float error = (float)WF_TARGET_DIST_MM - (float)dist_right;

        // Correct Logic: Near Wall -> Error Positive -> Positive Command ->
        // Left Turn Added Bias (+0.3f) to counteract Hardware Right Drift
        ang_cmd = (error * WF_KP) + 0.3f;
        lin_cmd = WF_BASE_SPEED;

      } else {
        // Priority 3: Lost Wall (Outer Corner) -> Turn Right
        // If we lose the wall, arc RIGHT to find it.

        lin_cmd = WF_BASE_SPEED * 0.8f;
        ang_cmd = -0.3f; // -0.3 rad/s (Right)
                         // ESP_LOGD(TAG, "Lost Wall: Right=%d", dist_right);
      }
    }

    // 4. Safety Clamp
    if (ang_cmd > WF_MAX_TURN_CMD)
      ang_cmd = WF_MAX_TURN_CMD;
    if (ang_cmd < -WF_MAX_TURN_CMD)
      ang_cmd = -WF_MAX_TURN_CMD;

    // 5. Send Command
    chassis_v2_set_velocity(lin_cmd, ang_cmd);

    // Update Stage (Simplified)
    // If we wanted to track stages, we'd do it here based on pos/history

    vTaskDelay(period);
  }
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
  // Ensure Vive Nav is stopped so it doesn't fight for motors
  vive_nav_stop();

  // DASH: Move forward 0.5m at 0.4m/s before starting Wall Following
  // This helps to leave the start area quickly
  // ESP_LOGI(TAG, "Wall Following START: Dashing 0.5m...");
  // chassis_v2_move_dist_blocking(0.5f, 0.4f);
  // ESP_LOGI(TAG, "Dash Complete. Activating Wall Following Logic.");

  g_status.state = WF2_RUNNING;
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
