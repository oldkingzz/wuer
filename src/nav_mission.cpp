/**
 * @file nav_mission.cpp
 * @brief Mission Control State Machine Implementation
 */

#include "include/nav_mission.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/chassis_v2.h"
#include "include/localization_ekf.h" // Added for ekf_get_pose
#include "include/vive_navigation.h"
#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "MISSION";

// ==========================================
// Global State
// ==========================================
static TaskHandle_t g_mission_task_handle = NULL;
static bool g_mission_running = false;
static nav_mission_status_t g_mission_status;
static uint32_t g_state_enter_time = 0;

// Helper to stop mission
static void nav_mission_stop_internal() {
  g_mission_running = false;
  g_mission_status.state = NAV_MISSION_STATE_IDLE;
  vive_nav_stop();
  chassis_v2_stop();
  ESP_LOGI(TAG, "Mission Stopped.");
}

// Helper to stop everything
esp_err_t nav_mission_stop(void) {
  nav_mission_stop_internal();
  return ESP_OK;
}

// ==========================================
// Mission Implementations
// ==========================================

// ==========================================
// Mission Implementations
// ==========================================

/**
 * @brief Helper: Point and Shoot Strategy
 * 1. Calculate angle from current EKF pose to (target_x, target_y)
 * 2. Turn to face target
 * 3. Calculate distance to target
 * 4. Move forward (distance + extra_run_m)
 * 5. Move backward (retreat_dist_m)
 */
static void mission_point_and_shoot_inch(float target_x_in, float target_y_in,
                                         float extra_run_m,
                                         float retreat_dist_m) {
  // 1. 获取当前位姿
  robot_pose_t pose = ekf_get_pose();

  // 2. 计算目标向量
  float dx_in = target_x_in - pose.x; // pose.x is in INCHES by EKF convention?
  // Wait, EKF convention usually METERS? Let's check localization_ekf.h or
  // usage. Codebase check: grid_map uses inches. ekf_init uses map_x
  // (pixels/inches). Assuming EKF state (x,y) are in **Inches** based on init
  // code in vive_navigation.cpp. Corrections: EKF internal state might be
  // meters if standard EKF. Re-reading vive_navigation.cpp:
  //   ekf_init(cx, cy, heading_rad); -> cx, cy are **pixels** (which are inches
  //   in map).
  // Yes, EKF state (x, y) is in **Inches**.

  float dy_in = target_y_in - pose.y;

  // 3. 计算目标角度 (atan2)
  float aim_angle_rad = atan2f(dy_in, dx_in);
  float aim_angle_deg = aim_angle_rad * 180.0f / M_PI;

  ESP_LOGI(TAG, "Point&Shoot: Aiming at (%.1f, %.1f) from (%.1f, %.1f)",
           target_x_in, target_y_in, pose.x, pose.y);
  ESP_LOGI(TAG, "  -> Angle: %.1f deg", aim_angle_deg);

  // 4. 转向瞄准
  chassis_v2_turn_angle_blocking(aim_angle_deg);
  vTaskDelay(pdMS_TO_TICKS(200)); // 稍作停顿稳定

  // 5. 计算距离并冲刺
  // Update pose after turn for better precision? Maybe unnecessary if turn is
  // in-place.
  float dist_in = sqrtf(dx_in * dx_in + dy_in * dy_in);
  float dist_m = dist_in * 0.0254f; // Convert Inch to Meter

  float total_drive_m = dist_m + extra_run_m;

  ESP_LOGI(TAG, "  -> Dist: %.2fm + %.2fm = %.2fm", dist_m, extra_run_m,
           total_drive_m);

  chassis_v2_move_dist_blocking(total_drive_m, 0.15f); // 0.15m/s Attack
  vTaskDelay(pdMS_TO_TICKS(200));

  // 6. 后退
  if (retreat_dist_m > 0) {
    ESP_LOGI(TAG, "  -> Retreat %.2fm", retreat_dist_m);
    chassis_v2_move_dist_blocking(-retreat_dist_m, 0.15f);
  }
}

// M1: Low Tower (Goal 0)
// 撞击点设为灵活变量，默认为 (40, 65)
static void run_mission_m1_low_tower() {
  static int step = 0;
  // Configurable Target (Inches)
  const float TARGET_X = 40.0f;
  const float TARGET_Y = 65.0f;
  const float EXTRA_M = 0.10f; // 10cm over-travel

  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0:
    ESP_LOGI(TAG, "M1: Nav to (40, 40)");
    vive_nav_set_target_map(40, 40);
    step = 1;
    break;
  case 1:
    if (g_nav_status.state == NAV_STATE_ARRIVED)
      step = 2;
    break;
  case 2: // Waypoint 2: Stop close to target
    ESP_LOGI(TAG, "M1: Nav to (40, 60)");
    vive_nav_set_target_map(40, 60);
    step = 3;
    break;
  case 3:
    if (g_nav_status.state == NAV_STATE_ARRIVED)
      step = 4;
    break;
  case 4: // Dynamic Aim & Attack
    vive_nav_stop();
    mission_point_and_shoot_inch(TARGET_X, TARGET_Y, EXTRA_M, 0.15f);

    ESP_LOGI(TAG, "M1: Complete");
    nav_mission_stop();
    step = 0;
    break;
  }
}

// M2: Navigate to (41, 29) and Stop
static void run_mission_m2_nexus() {
  static int step = 0;

  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0:
    ESP_LOGI(TAG, "M2: Nav to (41, 29)");
    vive_nav_set_target_map(41, 29);
    step = 1;
    break;
  case 1:
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      vive_nav_stop();
      ESP_LOGI(TAG, "M2: Arrived at (41, 29). Complete.");
      nav_mission_stop();
      step = 0;
    }
    break;
  }
}

// M3: High Tower (Goal 2)
// Strategy: From North (Y=88), Rush Down South to Y=72, Turn Right, Attack Wall
// (X=0)
// M3: New Goal -> (8, 55) (High Tower / Ramp Area)
static void run_mission_m3_high_tower() {
  static int step = 0;

  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0:
    ESP_LOGI(TAG, "M3: Nav to (8, 55)");
    vive_nav_set_target_map(8, 55);
    step = 1;
    break;
  case 1:
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      vive_nav_stop();
      ESP_LOGI(TAG, "M3: Arrived at (8, 55). Complete.");
      nav_mission_stop();
      step = 0;
    }
    break;
  }
}
// M4: Complex (Goal 3) - Placeholder
static void run_mission_m4_complex() {
  // Define complex logic here
  ESP_LOGI(TAG, "M4 (Complex): Start");
  // Example: Square
  chassis_v2_move_dist_blocking(0.2f, 0.1f);
  chassis_v2_turn_angle_blocking(90.0f);
  chassis_v2_move_dist_blocking(0.2f, 0.1f);
  chassis_v2_turn_angle_blocking(180.0f);
  chassis_v2_move_dist_blocking(0.2f, 0.1f);
  chassis_v2_turn_angle_blocking(-90.0f);
  chassis_v2_move_dist_blocking(0.2f, 0.1f);
  ESP_LOGI(TAG, "M4: Complete");
  nav_mission_stop();
}

// M5: Dash (Goal 4)
static void run_mission_m5_dash() {
  ESP_LOGI(TAG, "M5 (Dash): Move 0.08m");
  chassis_v2_move_dist_blocking(0.08f, 0.1f);
  ESP_LOGI(TAG, "M5: Complete");
  nav_mission_stop();
}

// M6: Turn 90 (Goal 5)
// Target Heading 90
static void run_mission_m6_turn() {
  ESP_LOGI(TAG, "M6 (Turn 90): Turning to 90 deg");
  chassis_v2_turn_angle_blocking(90.0f);
  ESP_LOGI(TAG, "M6: Complete");
  nav_mission_stop();
}

// ==========================================
// Main Task
// ==========================================
static void nav_mission_task(void *pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(50); // 20Hz Logic

  while (1) {
    if (g_mission_running) {
      uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
      g_mission_status.state_elapsed_ms = now - g_state_enter_time;

      // Dispatch Mission
      switch (g_mission_status.goal_id) {
      case NAV_GOAL_0:
        run_mission_m1_low_tower();
        break;
      case NAV_GOAL_1:
        run_mission_m2_nexus();
        break;
      case NAV_GOAL_2:
        run_mission_m3_high_tower();
        break;
      case NAV_GOAL_3:
        run_mission_m4_complex();
        break;
      case NAV_GOAL_4:
        run_mission_m5_dash();
        break;
      case NAV_GOAL_5:
        run_mission_m6_turn();
        break;
      default:
        ESP_LOGW(TAG, "Unknown Mission Goal: %d", g_mission_status.goal_id);
        nav_mission_stop();
        break;
      }
    }
    vTaskDelay(period);
  }
}

// ==========================================
// Public API
// ==========================================
esp_err_t nav_mission_init(void) {
  if (g_mission_task_handle != NULL)
    return ESP_OK;

  xTaskCreatePinnedToCore(nav_mission_task, "NavMission", 4096, NULL, 5,
                          &g_mission_task_handle, 1);
  return ESP_OK;
}

esp_err_t nav_mission_start(nav_goal_id_t goal_id) {
  if (g_mission_running) {
    ESP_LOGW(TAG, "Mission already running!");
    return ESP_FAIL;
  }

  g_mission_status.state = NAV_MISSION_STATE_GOTO_PRE_POINT; // Default start
  g_mission_status.goal_id = goal_id;
  // sub_step removed
  g_state_enter_time = pdTICKS_TO_MS(xTaskGetTickCount());
  g_mission_running = true;

  ESP_LOGI(TAG, "Mission Started! Goal: %d", goal_id);
  return ESP_OK;
}

// Note: nav_mission_stop is already defined at the top of the file

esp_err_t nav_mission_get_status(nav_mission_status_t *status) {
  if (status) {
    *status = g_mission_status;
  }
  return ESP_OK;
}
