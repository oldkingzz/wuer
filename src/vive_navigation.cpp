/**
 * @file vive_navigation.cpp
 * @brief Navigation System V2 - Comprehensive Implementation
 *
 * Features:
 * 1. EKF Localization (Encoder + Vive)
 * 2. A* Path Planning (using grid_map)
 * 3. Pure Pursuit Motion Control (Constant Speed)
 * 4. State Machine for Missions
 */

#include "include/vive_navigation.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/astar.h"      // Path Planning
#include "include/chassis_v2.h" // UPDATED: Use V2
#include "include/grid_map.h"
#include "include/localization_ekf.h" // EKF Module
#include "include/nav_config.h"
#include "include/nav_mission.h" // Mission Control
#include "include/vive_sensor.h"
#include <math.h>

static const char *TAG = "NAV";

// ==========================================
// Constants & Tunings
// ==========================================
#define NAV_LOOKAHEAD_DIST_INCH 10.0f // Pure Pursuit Lookahead
#define NAV_GOAL_TOLERANCE_INCH 2.0f  // Stop when closer than this
#define NAV_YAW_Align_TOLERANCE 0.1f  // ~5.7 degrees

// USE CONSTANT VELOCITY FROM V2 MACROS
// 用户要求降低线速度到 0.1 以保证安全
#define NAV_CONST_LINEAR_SPEED 0.10f // Reduced from 0.5

#define NAV_TURN_SPEED 1.5f // rad/s

#define PID_KP_YAW 0.6f // Reduced gain for smoother turning (was 1.0)

// ==========================================
// Global State
// ==========================================
static vive_pose_t g_current_pose = {0, 0, 0.0f, false};
static bool g_has_first_fix = false;

// Navigation State
static nav_status_t g_nav_status = {.state = NAV_STATE_IDLE};
static path_t g_current_path = {0}; // Store the planned path

// Chassis Odometry History for EKF Delta
static float last_dist_left = 0.0f;
static float last_dist_right = 0.0f;

// ==========================================
// Helper Functions
// ==========================================

// Normalize angle to -PI ~ PI
static float normalize_angle(float angle) {
  while (angle > M_PI)
    angle -= 2.0f * M_PI;
  while (angle < -M_PI)
    angle += 2.0f * M_PI;
  return angle;
}

// Stop Chassis Wrapper
static void stop_chassis() { chassis_v2_stop(); }

// ==========================================
// Localization Loop (Run every 20ms)
// ==========================================
#include <Arduino.h>

// ==========================================
// Localization Loop (Run every 20ms)
// ==========================================
static void update_localization_step(float dt) {
  // 1. Update Chassis Odometry (V2)
  chassis_v2_update_odometry(dt);

  // 2. Get current Chassis Odom
  float cur_left_m, cur_right_m;
  chassis_v2_get_wheel_dist_m(&cur_left_m, &cur_right_m);

  float d_left_m = cur_left_m - last_dist_left;
  float d_right_m = cur_right_m - last_dist_right;

  last_dist_left = cur_left_m;
  last_dist_right = cur_right_m;

  ekf_predict(d_left_m * 39.37f, d_right_m * 39.37f);

  // 4. Vive Update
  vive_data_t v1 = {0}, v2 = {0};
  float cx = 0, cy = 0;
  bool valid_signal = false;

  if (vive_get_latest_all(&v1, &v2) == ESP_OK) {
    if (v1.valid && v2.valid) {
      int16_t x1, y1, x2, y2;
      grid_map_vive_to_pixel(v1.x, v1.y, &x1, &y1);
      grid_map_vive_to_pixel(v2.x, v2.y, &x2, &y2);

      cx = (x1 + x2) / 2.0f;
      cy = (y1 + y2) / 2.0f;

      // Reverted Swap: Coordinate system was correct originally.

      // Fix: User confirmed "Facing -X (180), behavior is -Y (-90)".
      // Current logic (atan2 + PI) produces 180 when Real is -90.
      // Gap is -270 deg (+90).
      // We need to shift log by -270.
      // New = (atan2 + PI) - 3PI/2 = atan2 - PI/2.
      float heading_rad = atan2f(y2 - y1, x2 - x1);
      heading_rad -= M_PI / 2.0f;

      if (!g_has_first_fix) {
        Serial.printf("NAV: First Fix! Map:(%.1f, %.1f)\n", cx, cy);
        ekf_init(cx, cy, heading_rad);
        g_has_first_fix = true;
      } else {
        ekf_update_vive(cx, cy, heading_rad);
      }
      valid_signal = true;
    }
  }

  // 5. Publish
  robot_pose_t p = ekf_get_pose();
  g_current_pose.x = (uint16_t)p.x;
  g_current_pose.y = (uint16_t)p.y;
  g_current_pose.heading = p.theta * 180.0f / M_PI;
  g_current_pose.valid = g_has_first_fix;

  // No 0-360 conversion, keep -180 to 180
  // User requested -180 to 180 range. Removing 0-360 conversion.
  // if (g_current_pose.heading < 0)
  //   g_current_pose.heading += 360.0f;

  // Update nav_status for Web API
  g_nav_status.current_map_pos.x = g_current_pose.x;
  g_nav_status.current_map_pos.y = g_current_pose.y;

  // Debug Print (Every 1 second approx)
  static int debug_cnt = 0;
  if (++debug_cnt > 50) {
    debug_cnt = 0;
    // Fix: Show Heading in degrees for user verification
    float heading_deg = g_current_pose.heading;
    Serial.printf("NAV_LOG: V1(%d,%d) V2(%d,%d) Map(%.1f, %.1f) EKF(%.1f, "
                  "%.1f) CurH:%.1f deg Valid:%d\n",
                  v1.x, v1.y, v2.x, v2.y, cx, cy, p.x, p.y, heading_deg,
                  valid_signal);
  }
}

// ==========================================
// Motion Control Loop (Run every 20ms if NAVIGATING)
// ==========================================
static void update_motion_control_step(void) {
  if (g_nav_status.state != NAV_STATE_NAVIGATING)
    return;
  if (!g_current_pose.valid) {
    stop_chassis(); // Safety
    return;
  }

  robot_pose_t pose = ekf_get_pose();

  // 1. Check if reached Final Goal
  float dx_finish = (float)g_nav_status.target_map_pos.x - pose.x;
  float dy_finish = (float)g_nav_status.target_map_pos.y - pose.y;
  float dist_finish = sqrtf(dx_finish * dx_finish + dy_finish * dy_finish);

  if (dist_finish < NAV_GOAL_TOLERANCE_INCH) {
    ESP_LOGI(TAG, "Goal Reached! dist=%.1f", dist_finish);
    stop_chassis();
    g_nav_status.state = NAV_STATE_ARRIVED;
    return;
  }

  // 2. Pure Pursuit: Find Lookahead Point
  int16_t target_x, target_y;
  if (astar_get_next_target(&g_current_path, (int16_t)pose.x, (int16_t)pose.y,
                            NAV_LOOKAHEAD_DIST_INCH, &target_x,
                            &target_y) != ESP_OK) {
    // Fallback: aim at final goal
    target_x = g_nav_status.target_map_pos.x;
    target_y = g_nav_status.target_map_pos.y;
  }

  // 3. Compute Control Command
  float dx = (float)target_x - pose.x;
  float dy = (float)target_y - pose.y;
  float target_heading = atan2f(dy, dx);
  float heading_err = normalize_angle(target_heading - pose.theta);

  float linear_cmd = 0.0f;
  float angular_cmd = 0.0f;

  // A. Turn in Place if error is large (>45 deg) - increased from 30 to reduce
  // oscillation
  // Smooth Control: Scale linear speed based on heading error
  // (Turn-while-driving)
  angular_cmd = heading_err * PID_KP_YAW;

  // Clamp Angular Speed (Safety)
  if (angular_cmd > NAV_TURN_SPEED)
    angular_cmd = NAV_TURN_SPEED;
  if (angular_cmd < -NAV_TURN_SPEED)
    angular_cmd = -NAV_TURN_SPEED;

  // Linear Speed Scaling:
  // 1. If error is small, drive fast.
  // 2. If error is large (> 60 deg), stop and turn in place.
  // 3. In between, scale speed by cos(err).

  // Linear Speed Scaling:
  // ALWAYS move. Don't stop.
  // Scale speed by cos(err) to slow down in tight turns, but never 0.
  // Unless error > 90 (wrong way), then we naturally slow down or stop via
  // cos().

  float scale = cosf(heading_err);
  // Ensure we don't go backwards if error > 90
  if (scale < 0)
    scale = 0;

  // Minimal speed to prevent getting stuck if scaling is too aggressive?
  // User wants "Flow". Let's just use the cosine scale.
  linear_cmd = NAV_CONST_LINEAR_SPEED * scale;

  // Debug Print
  static int motion_log_timer = 0;
  if (++motion_log_timer >= 10) { // Every 200ms
    float cur_heading_deg = pose.theta * 180.0f / M_PI;
    float tgt_heading_deg = target_heading * 180.0f / M_PI;
    float err_deg = heading_err * 180.0f / M_PI;

    // User requested explicit Heading Debugging
    Serial.printf(
        "NAV_MOTION: St=%d Pos(%.1f,%.1f) TgtPos(%.1f,%.1f) | HEADINGS: "
        "Cur=%.1f Tgt=%.1f Err=%.1f | Cmd(L=%.2f, A=%.2f)\n",
        g_nav_status.state, pose.x, pose.y, (float)target_x, (float)target_y,
        cur_heading_deg, tgt_heading_deg, err_deg, linear_cmd, angular_cmd);
    motion_log_timer = 0;
  }

  // 4. Send to Chassis V2
  chassis_v2_set_velocity(linear_cmd, angular_cmd);
}

// ==========================================
// Main Task
// ==========================================
static void navigation_update_task(void *pvParameters) {
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20);

  int print_timer = 0;
  ESP_LOGI(TAG, "Nav Task Started (Loc+Motion)");

  while (1) {
    // 1. Localization
    update_localization_step(0.02f);

    // 2. Motion Control
    update_motion_control_step();

    // 3. Debug Print
    if (++print_timer >= 50) {
      robot_pose_t p = ekf_get_pose();
      ESP_LOGI(TAG, "State:%d P:%.1f,%.1f T:%.1f", g_nav_status.state, p.x, p.y,
               p.theta * 180.0f / M_PI);
      print_timer = 0;
    }

    vTaskDelayUntil(&last_wake, period);
  }
}

// ==========================================
// API Implementation
// ==========================================

esp_err_t vive_nav_init(void) {
  ESP_LOGI(TAG, "Init...");
  grid_map_init();
  grid_map_calibrate_affine(g_nav_calib_points, NAV_CALIB_POINT_COUNT);
  astar_init(); // Init A* Memory

  // Chassis V2 Init is done in wuer.ino, but we can reset odom here
  chassis_v2_reset_odometry();

  ekf_init(0, 0, 0);
  g_nav_status.state = NAV_STATE_IDLE; // Init State

  // Init Subsystems
  nav_mission_init();

  xTaskCreatePinnedToCore(navigation_update_task, "nav_upd", 4096, NULL, 5,
                          NULL, 1);
  return ESP_OK;
}

// ---------------------------------------------------------
// Set Target MAP (Trigger A* Planning)  <-- Main Logic Here
// ---------------------------------------------------------
esp_err_t vive_nav_set_target_map(int16_t map_x, int16_t map_y) {
  Serial.printf("NAV: Set Target Map: (%d, %d)\n", map_x, map_y);
  ESP_LOGI(TAG, "Set Target Map: (%d, %d)", map_x, map_y);

  if (!g_has_first_fix) {
    Serial.printf("NAV ERROR: No Localization Fix yet!\n");
    ESP_LOGE(TAG, "Ignored: No Localization Fix yet");
    return ESP_FAIL;
  }

  robot_pose_t start = ekf_get_pose();
  Serial.printf("NAV: Start pose: (%.1f, %.1f)\n", start.x, start.y);

  // 1. Plan Path
  esp_err_t ret = astar_plan_path((int16_t)start.x, (int16_t)start.y, map_x,
                                  map_y, &g_current_path);

  if (ret != ESP_OK) {
    Serial.printf("NAV ERROR: A* Plan Failed!\n");
    ESP_LOGE(TAG, "A* Plan Failed!");
    g_nav_status.state = NAV_STATE_ERROR;
    return ESP_FAIL;
  }

  // 2. Smooth Path
  astar_simplify_path(&g_current_path);

  // 3. Update Status
  g_nav_status.target_map_pos.x = map_x;
  g_nav_status.target_map_pos.y = map_y;
  g_nav_status.state = NAV_STATE_NAVIGATING; // Start Moving!
  g_nav_status.path_length = g_current_path.length;

  Serial.printf("NAV: Path Planned! Length: %d. Moving...\n",
                g_current_path.length);
  ESP_LOGI(TAG, "Path Planned! Length: %d. Moving...", g_current_path.length);
  return ESP_OK;
}

// Wrapper for Vive coordinates
esp_err_t vive_nav_set_target(uint16_t target_x, uint16_t target_y) {
  int16_t map_x, map_y;
  grid_map_vive_to_pixel(target_x, target_y, &map_x, &map_y);
  return vive_nav_set_target_map(map_x, map_y);
}

// Helper to set target by ID
esp_err_t vive_nav_set_target_goal(nav_goal_id_t goal_id) {
  if (goal_id >= NAV_GOAL_COUNT)
    return ESP_FAIL;
  nav_goal_t goal = g_nav_goals[goal_id];
  return vive_nav_set_target_map(goal.map_x, goal.map_y);
}

// ---------------------------------------------------------
// Other Controls
// ---------------------------------------------------------

esp_err_t vive_nav_start(void) {
  if (g_current_path.valid && g_current_path.length > 0) {
    g_nav_status.state = NAV_STATE_NAVIGATING;
    return ESP_OK;
  }
  return ESP_FAIL;
}

esp_err_t vive_nav_stop(void) {
  g_nav_status.state = NAV_STATE_IDLE;
  stop_chassis();
  return ESP_OK;
}

esp_err_t vive_nav_get_status(nav_status_t *status) {
  if (status)
    *status = g_nav_status;
  return ESP_OK;
}

esp_err_t vive_nav_get_pose(vive_pose_t *pose) {
  if (pose)
    *pose = g_current_pose;
  return ESP_OK;
}

esp_err_t vive_nav_get_path(path_t *path) {
  if (path)
    *path = g_current_path;
  return ESP_OK;
}

// Placeholder for replan
esp_err_t vive_nav_replan(void) { return ESP_OK; }
