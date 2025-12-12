/**
 * @file nav_mission.cpp
 * @brief Mission Control State Machine Implementation
 */

#include "include/nav_mission.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/chassis_v2.h"
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
static nav_mission_status_t g_mission_status = {.state = NAV_MISSION_STATE_IDLE,
                                                .goal_id = NAV_GOAL_0,
                                                .nav_state = NAV_STATE_IDLE,
                                                .state_elapsed_ms = 0};

static bool g_mission_running = false;
static TaskHandle_t g_mission_task_handle = NULL;
static uint32_t g_state_enter_time = 0;

// ==========================================
// ==========================================
// Independent Mission Logic Functions
// ==========================================

// Helper for normalizing angle -pi to pi
static float normalize_angle(float angle) {
  while (angle > (float)M_PI)
    angle -= 2.0f * (float)M_PI;
  while (angle < (float)-M_PI)
    angle += 2.0f * (float)M_PI;
  return angle;
}

// M1: Low Tower Blue (ID 1)
static void run_mission_attack_low_tower_blue() {
  static int step = 0;

  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t nav_st;
  vive_nav_get_status(&nav_st);
  uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

  // Debug Print
  static int last_step_print = -1;
  if (step != last_step_print) {
    ESP_LOGI(TAG, "M1: Step %d", step);
    last_step_print = step;
  }

  switch (step) {
  case 0: // Waypoint 1: (40, 40)
    // Run Step-by-Step Navigation
    // if (vive_nav_drive_blocking(40, 40) == ESP_OK) {
    //   step = 1;
    // } else {
    //   ESP_LOGE(TAG, "M1: Failed to reach (40, 40)");
    //   nav_mission_stop();
    // }
    ESP_LOGI(TAG, "M1: Waypoint 1 blocking drive disabled. Skipping.");
    // Note: blocking call finishes when arrived. So we can skip "wait state"
    // But to minimize code changes, we can just jump to step 2?
    // Let's just remove the Wait state (Case 1) and jump to next target
    step = 2;
    break;

  case 1:
    // OBSOLETE
    step = 2;
    break;

  case 2: // Waypoint 2: (40, 62)
    ESP_LOGI(TAG, "M1: Navigating to (40, 62)...");
    // if (vive_nav_drive_blocking(40, 62) == ESP_OK) {
    //   step = 4; // Jump to Alignment
    // } else {
    //   nav_mission_stop();
    // }
    ESP_LOGI(TAG, "M1: Waypoint 2 blocking drive disabled. Skipping.");
    step = 4;
    break;

  case 3: // OBSOLETE
    step = 4;
    break;

  case 4: // Check Alignment: Target +90 deg (+Y)
  {
    float target_heading = M_PI / 2.0f; // +90 deg = +PI/2

    // nav_st.current_pose.heading is in Degrees. Convert to Rad.
    float current_heading = nav_st.current_pose.heading * M_PI / 180.0f;
    current_heading = normalize_angle(current_heading);

    float error = normalize_angle(target_heading - current_heading);
    float error_deg = error * 180.0f / M_PI;

    // Tolerance 20 degrees
    if (fabs(error_deg) < 20.0f) {
      chassis_v2_set_velocity(0, 0); // Stop rotation
      ESP_LOGI(TAG, "M1: Aligned. Error=%.2f deg. Starting Attack.", error_deg);
      step = 5;
    } else {
      // P-Control for alignment
      // Debug occasionally
      static uint32_t last_log = 0;
      if (now - last_log > 500) {
        ESP_LOGI(TAG, "M1 Aligning... Err=%.1f deg", error_deg);
        last_log = now;
      }

      float kp = 1.0f;
      float ang_cmd = error * kp;
      // Clamp
      if (ang_cmd > 1.0f)
        ang_cmd = 1.0f;
      if (ang_cmd < -1.0f)
        ang_cmd = -1.0f;
      // Deadband
      float min_rot = 0.15f;
      if (fabs(ang_cmd) < min_rot)
        ang_cmd = (ang_cmd > 0) ? min_rot : -min_rot;

      chassis_v2_set_velocity(0.0f, ang_cmd);
    }
  } break;

  case 5: // Attack: Forward 6cm
    ESP_LOGI(TAG, "M1: Forward 6cm");
    chassis_v2_move_dist_blocking(0.06f, 0.1f);
    step = 6;
    break;

  case 6: // Attack: Backward 6cm
    ESP_LOGI(TAG, "M1: Backward 6cm");
    chassis_v2_move_dist_blocking(-0.06f, 0.1f);
    step = 7;
    break;

  case 7: // Finish
    ESP_LOGI(TAG, "M1 Complete.");
    nav_mission_stop();
    break;
  }
}

static void run_mission_attack_high_tower_blue() {}
static void run_mission_attack_nexus_blue() {
  static int step = 0;

  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t nav_st;
  vive_nav_get_status(&nav_st);
  uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

  // Debug Print
  static int last_step_print = -1;
  if (step != last_step_print) {
    ESP_LOGI(TAG, "M3: Step %d", step);
    last_step_print = step;
  }

  float target_heading = -M_PI / 2.0f; // -90 deg (-Y)

  switch (step) {
  case 0: // Waypoint 1: (40, 40)
    // if (vive_nav_drive_blocking(40, 40) == ESP_OK) {
    //   step = 2; // Skip wait
    // } else {
    //   nav_mission_stop();
    // }
    ESP_LOGI(TAG, "M3: Waypoint 1 blocking drive disabled. Skipping.");
    step = 2;
    break;

  case 1: // Wait for Waypoint 1
    step = 2;
    break;

  case 2: // Waypoint 2: (40, 15)
    ESP_LOGI(TAG, "M3: Navigating to (40, 15)...");
    // if (vive_nav_drive_blocking(40, 15) == ESP_OK) {
    //   step = 4; // Jump to Align
    // } else {
    //   nav_mission_stop();
    // }
    ESP_LOGI(TAG, "M3: Waypoint 2 blocking drive disabled. Skipping.");
    step = 4;
    break;

  case 3: // Wait for Waypoint 2
    step = 4;
    break;

  case 4: // Check Alignment: Target -90 deg (-Y)
  {
    // nav_st.current_pose.heading is in Degrees. Convert to Rad.
    float current_heading = nav_st.current_pose.heading * M_PI / 180.0f;
    current_heading = normalize_angle(current_heading);

    float error = normalize_angle(target_heading - current_heading);
    float error_deg = error * 180.0f / M_PI;

    // Tolerance 20 degrees
    if (fabs(error_deg) < 20.0f) {
      chassis_v2_set_velocity(0, 0); // Stop rotation
      ESP_LOGI(TAG, "M3: Aligned. Error=%.2f deg. Starting Attack.", error_deg);
      step = 5;
    } else {
      // P-Control for alignment
      static uint32_t last_log = 0;
      if (now - last_log > 500) {
        ESP_LOGI(TAG, "M3 Aligning... Err=%.1f deg", error_deg);
        last_log = now;
      }

      float kp = 1.0f;
      float ang_cmd = error * kp;
      // Clamp and Deadband
      if (ang_cmd > 1.0f)
        ang_cmd = 1.0f;
      if (ang_cmd < -1.0f)
        ang_cmd = -1.0f;

      float min_rot = 0.15f;
      if (fabs(ang_cmd) < min_rot)
        ang_cmd = (ang_cmd > 0) ? min_rot : -min_rot;

      chassis_v2_set_velocity(0.0f, ang_cmd);
    }
  } break;

  case 5: // Attack: Forward 10cm
    ESP_LOGI(TAG, "M3: Forward 10cm");
    chassis_v2_move_dist_blocking(0.1f, 0.1f);
    step = 6;
    break;

  case 6: // Attack: Backward 10cm
    ESP_LOGI(TAG, "M3: Backward 10cm");
    chassis_v2_move_dist_blocking(-0.1f, 0.1f);
    step = 7;
    break;

  case 7: // Finish
    ESP_LOGI(TAG, "M3 Complete.");
    nav_mission_stop();
    break;
  }
}

// M2: Low Tower Red (ID 2)
static void run_mission_attack_low_tower_red() {
  static int step = 0;

  // Need 'now' for logs
  uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

  // Init State logic
  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t nav_st;
  vive_nav_get_status(&nav_st);

  // Debug Print State
  static int last_printed_step = -1;
  static uint32_t last_print_time = 0;
  if (step != last_printed_step || (now - last_print_time > 1000)) {
    Serial.printf("M2_DEBUG: Step=%d | NavState=%d\n", step, nav_st.state);
    last_printed_step = step;
    last_print_time = now;
  }

  switch (step) {
  case 0: // 1. Start Navigation
    if (vive_nav_set_target_map(40, 83) == ESP_OK) {
      step = 1;
    } else {
      nav_mission_stop(); // Error
    }
    break;

  case 1: // 2. Wait for Arrival (with AGGRESSIVE TAKEOVER)
  {
    // Manual Distance Check
    float dx = (float)nav_st.current_map_pos.x - 40.0f;
    float dy = (float)nav_st.current_map_pos.y - 83.0f;
    float dist_sq = dx * dx + dy * dy; // Squared distance

    // Threshold: 8 inches (squared = 64.0)
    // Current status is ~4.34 inches away, so this WILL trigger.
    bool close_enough = (dist_sq < 64.0f);

    if (nav_st.state == NAV_STATE_ARRIVED || close_enough) {
      // COMMENTED OUT TO PREVENT SERIAL DEADLOCK
      // Serial.printf("M2: Distance %.1f inches. AGGRESSIVE TAKEOVER.\n",
      // sqrtf(dist_sq));

      // HARD STOP: Suspend Navigation Task completely
      vive_nav_suspend_task();
      step = 2;
    }
  } break;

  case 2: // 3. Drive Forward Forever
    // Just send command. No logic.
    chassis_v2_set_velocity(0.1f, 0.0f);

    // KEEP ALIVE LOG
    /*
    static uint32_t last_m2_log = 0;
    if (now - last_m2_log > 500) {
      Serial.printf("M2_ALIVE: In Case 2 (Forward Mode). Sending 0.1 m/s\n");
      last_m2_log = now;
    }
    */
    break;
  }
}
static void run_mission_attack_high_tower_red() {}
static void run_mission_attack_nexus_red() {
  static int step = 0;

  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state = NAV_MISSION_STATE_ALIGN_TOF;
  }

  nav_status_t nav_st;
  vive_nav_get_status(&nav_st);
  uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

  // Debug Print
  static int last_step_print = -1;
  if (step != last_step_print) {
    ESP_LOGI(TAG, "M4: Step %d", step);
    last_step_print = step;
  }

  float target_heading = M_PI / 2.0f; // +90 deg (+Y)

  switch (step) {
  case 0: // Waypoint 1: (40, 100)
    // if (vive_nav_drive_blocking(40, 100) == ESP_OK) {
    //   step = 2; // Skip wait
    // } else {
    //   nav_mission_stop();
    // }
    ESP_LOGI(TAG, "M4: Waypoint 1 blocking drive disabled. Skipping.");
    step = 2;
    break;

  case 1: // Wait for Waypoint 1
    step = 2;
    break;

  case 2: // Waypoint 2: (40, 130)
    ESP_LOGI(TAG, "M4: Navigating to (40, 130)...");
    // if (vive_nav_drive_blocking(40, 130) == ESP_OK) {
    //   step = 4;
    // } else {
    //   nav_mission_stop();
    // }
    ESP_LOGI(TAG, "M4: Waypoint 2 blocking drive disabled. Skipping.");
    step = 4;
    break;

  case 3: // Wait for Waypoint 2
    step = 4;
    break;

  case 4: // Check Alignment: Target +90 deg (+Y)
  {
    float current_heading = nav_st.current_pose.heading * M_PI / 180.0f;
    current_heading = normalize_angle(current_heading);

    float error = normalize_angle(target_heading - current_heading);
    float error_deg = error * 180.0f / M_PI;

    // Tolerance 20 degrees
    if (fabs(error_deg) < 20.0f) {
      chassis_v2_set_velocity(0, 0); // Stop rotation
      ESP_LOGI(TAG, "M4: Aligned. Error=%.2f deg. Starting Attack.", error_deg);
      step = 5;
    } else {
      // P-Control for alignment
      static uint32_t last_log = 0;
      if (now - last_log > 500) {
        ESP_LOGI(TAG, "M4 Aligning... Err=%.1f deg", error_deg);
        last_log = now;
      }

      float kp = 1.0f;
      float ang_cmd = error * kp;
      // Clamp and Deadband
      if (ang_cmd > 1.0f)
        ang_cmd = 1.0f;
      if (ang_cmd < -1.0f)
        ang_cmd = -1.0f;

      float min_rot = 0.15f;
      if (fabs(ang_cmd) < min_rot)
        ang_cmd = (ang_cmd > 0) ? min_rot : -min_rot;

      chassis_v2_set_velocity(0.0f, ang_cmd);
    }
  } break;

  case 5: // Attack: Forward 10cm
    ESP_LOGI(TAG, "M4: Forward 10cm");
    chassis_v2_move_dist_blocking(0.1f, 0.1f);
    step = 6;
    break;

  case 6: // Attack: Backward 10cm
    ESP_LOGI(TAG, "M4: Backward 10cm");
    chassis_v2_move_dist_blocking(-0.1f, 0.1f);
    step = 7;
    break;

  case 7: // Finish
    ESP_LOGI(TAG, "M4 Complete.");
    nav_mission_stop();
    break;
  }
}

// ==========================================
// Main Task
// ==========================================
static void nav_mission_task(void *pvParameters) {
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(50); // 20Hz Logic

  while (1) {
    if (g_mission_running) {
      // DEBUG: Verify which mission is running
      static uint32_t last_loop_log = 0;
      uint32_t now_loop = pdTICKS_TO_MS(xTaskGetTickCount());
      if (now_loop - last_loop_log > 2000) {
        Serial.printf("MISSION LOOP: Running=%d GoalID=%d\n", g_mission_running,
                      g_mission_status.goal_id);
        last_loop_log = now_loop;
      }

      g_mission_status.state_elapsed_ms =
          pdTICKS_TO_MS(xTaskGetTickCount()) - g_state_enter_time;

      // Mapping:
      // M1 (ID 1): Low Blue
      // M2 (ID 2): Low Red
      // M3 (ID 3): Nexus Blue
      // M4 (ID 4): Nexus Red
      // M5 (ID 5): High Blue
      // M6 (ID 6): High Red

      switch ((int)g_mission_status.goal_id) {
      case 1:
        run_mission_attack_low_tower_blue();
        break;
      case 2:
        // Debug Log
        // ESP_LOGI(TAG, "Dispatching M2...");
        run_mission_attack_low_tower_red();
        break;
      case 3:
        run_mission_attack_nexus_blue();
        break;
      case 4:
        run_mission_attack_nexus_red();
        break;
      case 5:
        run_mission_attack_high_tower_blue();
        break;
      case 6:
        run_mission_attack_high_tower_red();
        break;

      default:
        ESP_LOGW(TAG, "Unknown Goal ID: %d", g_mission_status.goal_id);
        nav_mission_stop();
        break;
      }
    }

    vTaskDelayUntil(&last_wake, period);
  }
}

// ==========================================
// API Implementation
// ==========================================

esp_err_t nav_mission_init(void) {
  ESP_LOGI(TAG, "Initializing Mission Control...");
  g_mission_status.state = NAV_MISSION_STATE_IDLE;

  xTaskCreatePinnedToCore(nav_mission_task, "nav_miss", 6144, NULL, 4,
                          &g_mission_task_handle, 1);
  return ESP_OK;
}

// Renamed from mission_start to match header
esp_err_t nav_mission_start(nav_goal_id_t goal_id) {
  ESP_LOGI(TAG, "Starting Mission for Goal ID: %d", goal_id);

  // Stop any low-level motion first?
  vive_nav_stop();

  g_mission_status.goal_id = goal_id;
  g_mission_status.state =
      NAV_MISSION_STATE_GOTO_PRE_POINT; // Default start state
  g_state_enter_time = pdTICKS_TO_MS(xTaskGetTickCount());
  g_mission_status.state_elapsed_ms = 0;

  g_mission_running = true;
  return ESP_OK;
}

// Renamed from mission_stop to match header
esp_err_t nav_mission_stop(void) {
  ESP_LOGI(TAG, "Stopping Mission");
  g_mission_running = false;
  g_mission_status.state = NAV_MISSION_STATE_IDLE;

  // Also stop chassis/nav
  vive_nav_stop();
  chassis_v2_set_velocity(0, 0); // Safety stop
  return ESP_OK;
}

esp_err_t nav_mission_get_status(nav_mission_status_t *status) {
  if (status)
    *status = g_mission_status;
  return ESP_OK;
}
