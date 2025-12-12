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
  static uint32_t step_start_time = 0;

  if (g_mission_status.state == NAV_MISSION_STATE_GOTO_PRE_POINT) {
    step = 0;
    g_mission_status.state =
        NAV_MISSION_STATE_ALIGN_TOF; // Mark as started (reusing ALIGN_TOF
                                     // state)
  }

  uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
  nav_status_t nav_st;
  vive_nav_get_status(&nav_st);

  switch (step) {
  case 0: // Start Navigation to (40, 83)
    ESP_LOGI(TAG, "M1(LowBlue): Navigating to (40, 83)...");
    if (vive_nav_set_target_map(40, 83) == ESP_OK) {
      step = 1;
    } else {
      ESP_LOGE(TAG, "M1: Failed to set target!");
      nav_mission_stop();
    }
    break;

  case 1: // Wait for Arrival
    if (nav_st.state == NAV_STATE_ARRIVED) {
      ESP_LOGI(TAG, "M1: Arrived. Stopping.");
      vive_nav_stop();
      nav_mission_stop(); // Stop Mission Here
    }
    break;

  case 2: // Align to -90 deg
  {
    float target_heading = -M_PI / 2.0f; // -90 deg
    // nav_st.heading is Degree (0-360). Convert to Radian (-PI to PI)
    float current_heading_deg = nav_st.current_pose.heading;
    float current_heading = current_heading_deg * M_PI / 180.0f;
    current_heading = normalize_angle(current_heading);

    float error = normalize_angle(target_heading - current_heading);

    // Debug Log (Rate Limited)
    static uint32_t last_log = 0;
    if (now - last_log > 200) {
      ESP_LOGI(TAG, "M1 HeadErr: %.2f (T:%.2f C:%.2f)", error, target_heading,
               current_heading);
      Serial.printf("M1 HeadErr: %.2f (T:%.2f C:%.2f)\n", error, target_heading,
                    current_heading);
      last_log = now;
    }

    if (fabs(error) < 0.2f) { // ~11 deg tolerance
      chassis_v2_set_velocity(0, 0);
      step = 3;
      step_start_time = now;
      ESP_LOGI(TAG, "M1: Aligned. Error=%.2f", error);
    } else {
      // P Control - Reduced Gain
      float ang_cmd = error * 1.0f;
      if (ang_cmd > 1.0f)
        ang_cmd = 1.0f;
      if (ang_cmd < -1.0f)
        ang_cmd = -1.0f;
      // Deadband
      if (fabs(ang_cmd) < 0.1f)
        ang_cmd = (ang_cmd > 0) ? 0.1f : -0.1f;

      chassis_v2_set_velocity(0.0f, ang_cmd);
    }
  } break;

  case 3: // Dash Forward 6 inches (0.1524 m)
  {
    static float start_l = 0, start_r = 0;
    if (step_start_time == now)
      chassis_v2_get_wheel_dist_m(&start_l, &start_r);

    float curr_l, curr_r;
    chassis_v2_get_wheel_dist_m(&curr_l, &curr_r);
    float dist = ((curr_l - start_l) + (curr_r - start_r)) / 2.0f;

    if (dist >= 0.1524f) {
      chassis_v2_set_velocity(0, 0);
      step = 4;
      step_start_time = now;
      ESP_LOGI(TAG, "M1: Dash Done. Retreating...");
    } else {
      chassis_v2_set_velocity(0.1f, 0.0f);
    }
  } break;

  case 4: // Retreat 6 inches
  {
    static float start_l = 0, start_r = 0;
    if (step_start_time == now)
      chassis_v2_get_wheel_dist_m(&start_l, &start_r);

    float curr_l, curr_r;
    chassis_v2_get_wheel_dist_m(&curr_l, &curr_r);
    float dist = ((curr_l - start_l) + (curr_r - start_r)) / 2.0f;

    if (dist <= -0.1524f) {
      chassis_v2_set_velocity(0, 0);
      step = 5;
      ESP_LOGI(TAG, "M1: Retreat Done. Mission Complete.");
    } else {
      chassis_v2_set_velocity(-0.1f, 0.0f);
    }
  } break;

  case 5: // Finish
    nav_mission_stop();
    break;
  }
}

static void run_mission_attack_high_tower_blue() {}
static void run_mission_attack_nexus_blue() {}

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
static void run_mission_attack_nexus_red() {}

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
