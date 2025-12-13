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

// M1: Low Tower Blue (ID 1)
static void run_mission_attack_low_tower_blue() {
  static int step = 0;

  // RESET LOGIC
  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF; // Mark as running
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0: // Waypoint 1: (40, 40)
    ESP_LOGI(TAG, "M1: Step 0 -> Nav to (40, 40)");
    vive_nav_set_target_map(40, 40);
    step = 1;
    break;

  case 1: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 2;
    }
    break;

  case 2: // Waypoint 2: (40, 60) - Adjusted from 62 to prevent stuck
    ESP_LOGI(TAG, "M1: Step 2 -> Nav to (40, 60)");
    vive_nav_set_target_map(40, 60);
    step = 3;
    break;

  case 3: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 4;
    }
    break;

  case 4: // Align to +Y (+90 deg)
  {
    vive_nav_stop();
    robot_pose_t pose = ekf_get_pose();
    float target_rad = M_PI / 2.0f; // +90 deg
    float diff_rad = target_rad - pose.theta;
    while (diff_rad > M_PI)
      diff_rad -= 2 * M_PI;
    while (diff_rad < -M_PI)
      diff_rad += 2 * M_PI;

    ESP_LOGI(TAG, "M1: Aligning to +90 deg. Diff: %.1f deg",
             diff_rad * 180.0f / M_PI);
    chassis_v2_turn_angle_blocking(diff_rad * 180.0f / M_PI, 60.0f);
    step = 5;
  } break;

  case 5: // Attack 15cm
    ESP_LOGI(TAG, "M1: ATTACK! Forward 15cm");
    chassis_v2_move_dist_blocking(0.15f, 0.1f);
    vTaskDelay(pdMS_TO_TICKS(200));
    step = 6;
    break;

  case 6: // Retreat 15cm
    ESP_LOGI(TAG, "M1: Retreat! Back 15cm");
    chassis_v2_move_dist_blocking(-0.15f, 0.1f);
    step = 7;
    break;

  case 7: // Finish
    ESP_LOGI(TAG, "M1: Mission Complete");
    nav_mission_stop();
    step = 0;
    break;
  }
}

// M2: Low Tower Red (ID 2)
static void run_mission_attack_low_tower_red() {
  static int step = 0;

  // RESET LOGIC
  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF; // Mark as running
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0: // Waypoint 1: (27, 82)
    ESP_LOGI(TAG, "M2: Step 0 -> Nav to (27, 82)");
    vive_nav_set_target_map(27, 82);
    step = 1;
    break;

  case 1: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 2;
    }
    break;

  case 2: // Waypoint 2: (30, 51)
    ESP_LOGI(TAG, "M2: Step 2 -> Nav to (30, 51)");
    vive_nav_set_target_map(30, 51);
    step = 3;
    break;

  case 3: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 4;
    }
    break;

  case 4: // Align to -Y (-90 deg)
  {
    vive_nav_stop();
    robot_pose_t pose = ekf_get_pose();
    float target_rad = -M_PI / 2.0f; // -90 deg
    float diff_rad = target_rad - pose.theta;
    while (diff_rad > M_PI)
      diff_rad -= 2 * M_PI;
    while (diff_rad < -M_PI)
      diff_rad += 2 * M_PI;

    ESP_LOGI(TAG, "M2: Aligning to -90 deg. Diff: %.1f deg",
             diff_rad * 180.0f / M_PI);
    chassis_v2_turn_angle_blocking(diff_rad * 180.0f / M_PI, 60.0f);
    step = 5;
  } break;

  case 5: // Attack 15cm
    ESP_LOGI(TAG, "M2: ATTACK! Forward 15cm");
    chassis_v2_move_dist_blocking(0.15f, 0.1f);
    vTaskDelay(pdMS_TO_TICKS(200));
    step = 6;
    break;

  case 6: // Retreat 15cm
    ESP_LOGI(TAG, "M2: Retreat! Back 15cm");
    chassis_v2_move_dist_blocking(-0.15f, 0.1f);
    step = 7;
    break;

  case 7: // Finish
    ESP_LOGI(TAG, "M2: Mission Complete");
    nav_mission_stop();
    step = 0;
    break;
  }
}

// M3: Nexus Blue (ID 3)
static void run_mission_attack_nexus_blue() {
  static int step = 0;
  static int attack_count = 0; // Added for repeat attack

  // RESET LOGIC
  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF; // Mark as running
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0: // Waypoint 1: (40, 40)
    ESP_LOGI(TAG, "M3: Step 0 -> Nav to (40, 40)");
    vive_nav_set_target_map(40, 40);
    step = 1;
    break;

  case 1: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 2;
    }
    break;

  case 2: // Waypoint 2: (40, 30)
    ESP_LOGI(TAG, "M3: Step 2 -> Nav to (40, 30)");
    vive_nav_set_target_map(40, 30);
    step = 3;
    break;

  case 3: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 4;
    }
    break;

  case 4: // Precise Align Sequence (X-Fix then Attack Angle)
  {
    vive_nav_stop();

    // 1. Turn to 0 deg (+X)
    robot_pose_t pose = ekf_get_pose();
    float diff_to_0 = 0.0f - pose.theta;
    while (diff_to_0 > M_PI)
      diff_to_0 -= 2 * M_PI;
    while (diff_to_0 < -M_PI)
      diff_to_0 += 2 * M_PI;
    ESP_LOGI(TAG, "M3: Turn to 0 deg for X-Fix");
    chassis_v2_turn_angle_blocking(diff_to_0 * 180.0f / M_PI, 60.0f);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 2. Fix X Error
    pose = ekf_get_pose();
    float target_x_m = 40.0f * 0.0254f;
    float x_err_m = target_x_m - pose.x;
    ESP_LOGI(TAG, "M3: X-Fix Err: %.3fm", x_err_m);
    if (fabs(x_err_m) > 0.01f) {
      chassis_v2_move_dist_blocking(x_err_m, 0.1f);
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    // 3. Turn to -90 deg (-Y)
    pose = ekf_get_pose();
    float target_rad = -M_PI / 2.0f;
    float diff_rad = target_rad - pose.theta;
    while (diff_rad > M_PI)
      diff_rad -= 2 * M_PI;
    while (diff_rad < -M_PI)
      diff_rad += 2 * M_PI;

    ESP_LOGI(TAG, "M3: Final Align to -90 deg");
    chassis_v2_turn_angle_blocking(diff_rad * 180.0f / M_PI, 60.0f);

    attack_count = 0; // Reset counter
    step = 5;
  } break;

  case 5: // Attack 5cm
    ESP_LOGI(TAG, "M3: ATTACK! Forward 5cm (Round %d/4)", attack_count + 1);
    chassis_v2_move_dist_blocking(0.05f, 0.1f);
    vTaskDelay(pdMS_TO_TICKS(100)); // Short delay
    step = 6;
    break;

  case 6: // Retreat 5cm
    ESP_LOGI(TAG, "M3: Retreat! Back 5cm");
    chassis_v2_move_dist_blocking(-0.05f, 0.1f); // Faster retreat
    attack_count++;
    if (attack_count < 4) {
      step = 5; // Repeat
    } else {
      step = 7; // Done
    }
    break;

  case 7: // Finish
    ESP_LOGI(TAG, "M3: Mission Complete");
    nav_mission_stop();
    step = 0;
    break;
  }
}

// M4: Nexus Red (ID 4)
static void run_mission_attack_nexus_red() {
  static int step = 0;
  static int attack_count = 0; // Added for repeat

  // RESET LOGIC
  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF; // Mark as running
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0: // Waypoint 1: (40, 100)
    ESP_LOGI(TAG, "M4: Step 0 -> Nav to (40, 100)");
    vive_nav_set_target_map(40, 100);
    step = 1;
    break;

  case 1: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 2;
    }
    break;

  case 2: // Waypoint 2: (40, 115)
    ESP_LOGI(TAG, "M4: Step 2 -> Nav to (40, 115)");
    vive_nav_set_target_map(40, 115);
    step = 3;
    break;

  case 3: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 4;
    }
    break;

  case 4: // Precise Align Sequence (X-Fix then Attack Angle)
  {
    vive_nav_stop();

    // 1. Turn to 0 deg (+X)
    robot_pose_t pose = ekf_get_pose();
    float diff_to_0 = 0.0f - pose.theta;
    while (diff_to_0 > M_PI)
      diff_to_0 -= 2 * M_PI;
    while (diff_to_0 < -M_PI)
      diff_to_0 += 2 * M_PI;
    ESP_LOGI(TAG, "M4: Turn to 0 deg for X-Fix");
    chassis_v2_turn_angle_blocking(diff_to_0 * 180.0f / M_PI, 60.0f);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 2. Fix X Error
    pose = ekf_get_pose();
    float target_x_m = 40.0f * 0.0254f;
    float x_err_m = target_x_m - pose.x;
    ESP_LOGI(TAG, "M4: X-Fix Err: %.3fm", x_err_m);
    if (fabs(x_err_m) > 0.01f) {
      chassis_v2_move_dist_blocking(x_err_m, 0.1f);
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    // 3. Turn to +90 deg (+Y)
    pose = ekf_get_pose();
    float target_rad = M_PI / 2.0f;
    float diff_rad = target_rad - pose.theta;
    while (diff_rad > M_PI)
      diff_rad -= 2 * M_PI;
    while (diff_rad < -M_PI)
      diff_rad += 2 * M_PI;

    ESP_LOGI(TAG, "M4: Final Align to +90 deg");
    chassis_v2_turn_angle_blocking(diff_rad * 180.0f / M_PI, 60.0f);

    attack_count = 0; // Reset counter
    step = 5;
  } break;

  case 5: // Attack 5cm
    ESP_LOGI(TAG, "M4: ATTACK! Forward 5cm (Round %d/4)", attack_count + 1);
    chassis_v2_move_dist_blocking(0.05f, 0.1f);
    vTaskDelay(pdMS_TO_TICKS(100));
    step = 6;
    break;

  case 6: // Retreat 5cm
    ESP_LOGI(TAG, "M4: Retreat! Back 5cm");
    chassis_v2_move_dist_blocking(-0.05f, 0.1f);
    attack_count++;
    if (attack_count < 4) {
      step = 5;
    } else {
      step = 7;
    }
    break;

  case 7: // Finish
    ESP_LOGI(TAG, "M4: Mission Complete");
    nav_mission_stop();
    step = 0;
    break;
  }
}

// M5: Blue Challenge (ID 5)
// Path: (23,117) -> (6.5,118) -> (9.3,53)[HighSpeed] -> Turn CW(-180) ->
// Check -> Act -> Turn CCW(-90) -> (6.5,47) -> (24,46.5)
static void run_mission_M5() {
  static int step = 0;

  // RESET LOGIC
  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0: // Waypoint 1: Blue Point (9, 53) - Direct Travel
    ESP_LOGI(TAG, "M5: Step 0 -> Nav to Blue Point (9, 53)");
    vive_nav_set_target_map(9, 53);
    step = 1;
    break;

  case 1: // Wait Arrival
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 2;
    }
    break;

  case 2: // Turn CW 90
    ESP_LOGI(TAG, "M5: Turn CW 90");
    vive_nav_stop();
    chassis_v2_turn_angle_blocking(-90.0f, 60.0f);
    step = 3;
    break;

  case 3: // Attack
    ESP_LOGI(TAG, "M5: Attack (6cm)");
    chassis_v2_move_dist_blocking(0.06f, 0.1f);
    vTaskDelay(pdMS_TO_TICKS(200));
    step = 5; // Go to Finish (Skip Retreat)
    break;

  case 5: // Finish
    ESP_LOGI(TAG, "M5 Complete");
    nav_mission_stop();
    step = 0;
    break;
  }
}

// M6: Red Challenge (ID 6)
// Path: (24,46.5) -> (6.5,47) -> (5.8,78)[HighSpeed] -> Turn CCW -> Check
// -> Act -> Turn CW -> (6.5,118) -> (23,117)
static void run_mission_M6() {
  static int step = 0;

  // RESET LOGIC
  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t g_nav_status;
  vive_nav_get_status(&g_nav_status);

  switch (step) {
  case 0: // Waypoint 1: (8.4, 46.5) -> (8, 46) Left Lower Entry
    ESP_LOGI(TAG, "M6: Step 0 -> Nav to (8, 46)");
    vive_nav_set_target_map(8, 46);
    step = 1;
    break;

  case 1:
    if (g_nav_status.state == NAV_STATE_ARRIVED)
      step = 2;
    break;

  case 2: // Waypoint 2: (6.5, 47) Left Lower Slope
    ESP_LOGI(TAG, "M6: Step 2 -> Nav to (6.5, 47)");
    vive_nav_set_target_map(6, 47);
    step = 3;
    break;

  case 3:
    if (g_nav_status.state == NAV_STATE_ARRIVED)
      step = 4;
    break;

  case 4: // Waypoint 3: (5.8, 78) Red Point [HighSpeed]
    ESP_LOGI(TAG, "M6: Step 4 -> Nav to (5.8, 78)");
    // Speed up removed
    vive_nav_set_target_map(6, 78);
    step = 5;
    break;

  case 5:
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      step = 6;
    }
    break;

  case 6: // Turn CCW 90 deg. Current +90 (+Y). Goal 180 (-X).
    ESP_LOGI(TAG, "M6: Turn CCW 90");
    chassis_v2_turn_angle_blocking(90.0f, 60.0f);
    step = 7;
    break;

  case 7: // Check Heading (-X -> 180). Tol 15 deg.
  {
    vive_nav_stop();
    robot_pose_t pose = ekf_get_pose();
    float target_rad = M_PI; // 180 deg
    float diff_rad = target_rad - pose.theta;
    while (diff_rad > M_PI)
      diff_rad -= 2 * M_PI;
    while (diff_rad < -M_PI)
      diff_rad += 2 * M_PI;

    float err_deg = diff_rad * 180.0f / M_PI;
    ESP_LOGI(TAG, "M6: Checking Heading -X. Err: %.1f", err_deg);

    if (fabs(err_deg) > 15.0f) {
      ESP_LOGI(TAG, "M6: Correcting Heading...");
      chassis_v2_turn_angle_blocking(err_deg, 45.0f);
    }
    step = 8;
  } break;

  case 8: // Action: Fwd 4cm, Back 4cm
    ESP_LOGI(TAG, "M6: Action 4cm");
    chassis_v2_move_dist_blocking(0.04f, 0.1f);
    chassis_v2_move_dist_blocking(-0.04f, 0.1f);
    step = 9;
    break;

  case 9: // Turn CW 90 (-90). Target back to +90.
    ESP_LOGI(TAG, "M6: Turn CW 90");
    chassis_v2_turn_angle_blocking(-90.0f, 60.0f);
    step = 10;
    break;

  case 10: // Waypoint 4: (6.5, 118) Left Upper Slope
    ESP_LOGI(TAG, "M6: Step 10 -> Nav to (6.5, 118)");
    vive_nav_set_target_map(6, 118);
    step = 11;
    break;

  case 11:
    if (g_nav_status.state == NAV_STATE_ARRIVED)
      step = 12;
    break;

  case 12: // Waypoint 5: (23, 117) Left Upper Entry
    ESP_LOGI(TAG, "M6: Step 12 -> Nav to (23, 117)");
    vive_nav_set_target_map(23, 117);
    step = 13;
    break;

  case 13:
    if (g_nav_status.state == NAV_STATE_ARRIVED) {
      ESP_LOGI(TAG, "M6 Complete");
      nav_mission_stop();
    }
    break;
  }
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
      case NAV_GOAL_0: // Low Tower Blue (Web Button 0)
        run_mission_attack_low_tower_blue();
        break;
      case NAV_GOAL_1: // Low Tower Red (Web Button 1)
        run_mission_attack_low_tower_red();
        break;
      case NAV_GOAL_2: // Nexus Blue (Web Button 2)
        run_mission_attack_nexus_blue();
        break;
      case NAV_GOAL_3: // Nexus Red (Web Button 3)
        run_mission_attack_nexus_red();
        break;
      case NAV_GOAL_4: // Blue Challenge (Web Button 4)
        run_mission_M5();
        break;
      case NAV_GOAL_5: // Red Challenge (Web Button 5)
        run_mission_M6();
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
