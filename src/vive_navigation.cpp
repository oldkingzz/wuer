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
#define NAV_GOAL_TOLERANCE_INCH 2.5f  // User Requested 2.5
#define NAV_YAW_Align_TOLERANCE 0.1f  // ~5.7 degrees

// USE CONSTANT VELOCITY FROM V2 MACROS
// 用户要求 (0.25太快 -> 0.20)
#define NAV_CONST_LINEAR_SPEED 0.20f

#define NAV_TURN_SPEED 1.5f // rad/s

// PID Parameters for Heading
// 0.2m/s 对应 Kp=1.2 (适中)
#define PID_KP_YAW 1.2f

// ==========================================
// Global State
// ==========================================
static vive_pose_t g_current_pose = {0, 0, 0.0f, false};
static bool g_has_first_fix = false;

// Auto-Routing State for Via-Point Sequence
static bool s_has_pending_final = false;
static int16_t s_pending_final_x = 0;
static int16_t s_pending_final_y = 0;

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
      heading_rad =
          normalize_angle(heading_rad); // CRITICAL FIX: Normalize immediately

      if (!g_has_first_fix) {
        Serial.printf("NAV: First Fix! Map:(%.1f, %.1f)\n", cx, cy);
        ekf_init(cx, cy, heading_rad);
        g_has_first_fix = true;
      } else {
        // --- Outlier Rejection (Gatekeeper) ---
        robot_pose_t current_est = ekf_get_pose();
        float dist_err =
            sqrtf(powf(cx - current_est.x, 2) + powf(cy - current_est.y, 2));

        // Threshold: 15.0 pixel/inch (Conservative jump limit)
        // If jump is huge, ignore it, UNLESS it persists (Kidnapped robot)
        static int s_reject_count = 0;
        const int MAX_REJECT_COUNT = 20; // ~400ms of bad data

        if (dist_err > 15.0f && s_reject_count < MAX_REJECT_COUNT) {
          s_reject_count++;
          // Log occasionally
          if (s_reject_count % 5 == 0) {
            ESP_LOGW(TAG, "Vive Glitch Ignored! Dist=%.1f. Cnt=%d", dist_err,
                     s_reject_count);
          }
          // DO NOT UPDATE EKF with this bad data
        } else {
          if (s_reject_count >= MAX_REJECT_COUNT) {
            ESP_LOGW(TAG, "Vive Jump Persisted! Force Updating EKF.");
          }
          s_reject_count = 0; // Reset counter
          ekf_update_vive(cx, cy, heading_rad);
        }
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
  // STRICTLY EXIT if not navigating. Do NOT touch motors.
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
    if (s_has_pending_final) {
      ESP_LOGI(TAG, "Via-Point Reached! Continuing to Next Goal (%d, %d)...",
               s_pending_final_x, s_pending_final_y);
      // Trigger next leg immediately
      int16_t next_x = s_pending_final_x;
      int16_t next_y = s_pending_final_y;
      // Important: clear flag BEFORE calling set_target to prevent infinite
      // recursion loop logic
      s_has_pending_final = false;
      vive_nav_set_target_map(next_x, next_y);
      return; // Continue navigating
    }

    ESP_LOGI(TAG, "Goal Reached! dist=%.1f", dist_finish);
    stop_chassis();
    g_nav_status.state = NAV_STATE_ARRIVED;
    return;
  }

  // 2. Simple Heading Tracking
  int16_t target_x, target_y;
  uint16_t next_index = 0;
  // Use a small lookahead just to get the immediate next direction
  // Pass current_waypoint as start_index to prevent backtracking
  if (astar_get_next_target(
          &g_current_path, (int16_t)pose.x, (int16_t)pose.y,
          12.0f, // 12 inches ahead (Smoother, Straighter lines)
          g_nav_status.current_waypoint, &target_x, &target_y,
          &next_index) != ESP_OK) {
    target_x = g_nav_status.target_map_pos.x;
    target_y = g_nav_status.target_map_pos.y;
  } else {
    // Only update forward (monotonicity check inside astar already done by
    // start_index, but good to be safe)
    if (next_index > g_nav_status.current_waypoint) {
      g_nav_status.current_waypoint = next_index;
    }
  }

  // 3. The Core Logic: It is JUST a Heading Angle.
  float dx = (float)target_x - pose.x;
  float dy = (float)target_y - pose.y;
  float target_heading = atan2f(dy, dx);
  float current_heading = pose.theta;
  float heading_err = normalize_angle(target_heading - current_heading);

  // 4. Control: Fix the Angle, Drive Forward
  // P-Controller for Turn
  float angular_cmd = heading_err * PID_KP_YAW;

  // Constant Speed (stop if facing wrong way)
  // Constant Speed (stop if facing wrong way)
  // STRICT MODE: Stop and turn if error is > 10 degrees.
  // This mimicks "Step-by-Step" navigation: Align perfectly first, then move.
  float linear_cmd = NAV_CONST_LINEAR_SPEED;
  if (fabsf(heading_err) > (10.0f * M_PI / 180.0f)) {
    linear_cmd = 0.0f; // Turn in place first
  }

  // Debug
  static int motion_log_timer = 0;
  if (++motion_log_timer >= 20) {
    Serial.printf("NAV_SIMPLE: Tgt(%.0f,%.0f) | HEAD: Tgt=%.1f Cur=%.1f "
                  "Err=%.1f | Cmd: L=%.2f A=%.2f\n",
                  (float)target_x, (float)target_y,
                  target_heading * 180.0f / M_PI,
                  current_heading * 180.0f / M_PI, heading_err * 180.0f / M_PI,
                  linear_cmd, angular_cmd);
    motion_log_timer = 0;
  }

  chassis_v2_set_velocity(linear_cmd, angular_cmd);
}

// ==========================================
// Main Task
// ==========================================
// Task Handle (Not used for Suspend anymore to avoid Deadlock)
static TaskHandle_t g_nav_task_handle = NULL;
static volatile bool g_nav_paused = false; // Safe Pause Flag

void vive_nav_suspend_task(void) {
  // SOFT STOP: Don't kill the task, just tell it to sleep.
  // This prevents Serial/I2C deadlocks.
  g_nav_paused = true;
  chassis_v2_set_velocity(0, 0);
  ESP_LOGW(TAG, "NAV TASK PAUSED (Safe Soft Stop)");
}

void vive_nav_resume_task(void) {
  g_nav_paused = false;
  ESP_LOGI(TAG, "NAV TASK RESUMED");
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
    // 0. SAFE PAUSE CHECK
    if (g_nav_paused) {
      static TickType_t last_pause_log = 0;
      TickType_t now_tick = xTaskGetTickCount();
      if (now_tick - last_pause_log > pdMS_TO_TICKS(2000)) {
        // USER REQUESTED EXPLICIT LOG
        printf("NAV TASK: I AM PAUSED. SLEEPING... (Not touching motors)\n");
        last_pause_log = now_tick;
      }
      vTaskDelay(pdMS_TO_TICKS(100)); // Sleep 100ms
      continue;                       // Skip everything
    }

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

  // Ensure Nav task priority (3) is LOWER than Mission task (4)
  // so Mission can override/control commands without fighting for CPU time.
  xTaskCreatePinnedToCore(navigation_update_task, "nav_upd", 4096, NULL, 3,
                          &g_nav_task_handle, 1);
  return ESP_OK;
}

// ---------------------------------------------------------
// Set Target MAP (Trigger A* Planning)  <-- Main Logic Here
// ---------------------------------------------------------
esp_err_t vive_nav_set_target_map(int16_t map_x, int16_t map_y) {
  // 0. Safety Check
  if (!g_has_first_fix) {
    Serial.printf("NAV ERROR: No Localization Fix yet!\n");
    ESP_LOGE(TAG, "Ignored: No Localization Fix yet");
    return ESP_FAIL;
  }

  robot_pose_t start = ekf_get_pose();

  // 1. Zone Crossing Check (Auto-Routing via 28, 69)
  // Define Boundary: Y = 69
  // Logic: If Start and End are on opposite sides of Y=69, Force Via-Point.
  // Exception: If we are already very close to Via-Point (< 8 inches), just go
  // direct.

  // Reset pending by default (unless we set it below)
  // Note: If this function was called BY the pending logic, s_has_pending_final
  // was cleared just before. So we default to clearing it for new user
  // commands. Assuming this function is not re-entrant.

  // Actually, we should check if this call IS the "next leg".
  // The caller (motion loop) clears the flag before calling.
  // So if we are called with s_has_pending_final == false, it's a fresh
  // command.

  bool uses_via_point = false;
  if (!s_has_pending_final) { // Only check for fresh commands
    // Define Threshold and Via Points
    const int16_t SPLIT_Y = 73;
    const int16_t VIA_UPPER_X = 30;
    const int16_t VIA_UPPER_Y = 80;
    const int16_t VIA_LOWER_X = 30;
    const int16_t VIA_LOWER_Y = 66;

    bool start_upper = (start.y > SPLIT_Y);
    bool target_upper = (map_y > SPLIT_Y);

    if (start_upper != target_upper) {
      // Crossing Zones Detected.
      // Logic: Start -> Entry Via -> Exit Via -> Final

      int16_t entry_x = start_upper ? VIA_UPPER_X : VIA_LOWER_X;
      int16_t entry_y = start_upper ? VIA_UPPER_Y : VIA_LOWER_Y;

      int16_t exit_x = start_upper ? VIA_LOWER_X : VIA_UPPER_X;
      int16_t exit_y = start_upper ? VIA_LOWER_Y : VIA_UPPER_Y;

      float dist_to_entry =
          sqrtf(powf(start.x - entry_x, 2) + powf(start.y - entry_y, 2));

      float dist_to_exit =
          sqrtf(powf(start.x - exit_x, 2) + powf(start.y - exit_y, 2));

      // 1. If not at Entry yet, go to Entry
      if (dist_to_entry > 8.0f) {
        Serial.printf("NAV: Crossing Zone. Step 1: Go to Entry (%d, %d)\n",
                      entry_x, entry_y);
        s_pending_final_x = map_x;
        s_pending_final_y = map_y;
        s_has_pending_final = true;

        map_x = entry_x;
        map_y = entry_y;
        uses_via_point = true;
      }
      // 2. If at Entry (or close) but not at Exit, go to Exit
      else if (dist_to_exit > 8.0f) {
        Serial.printf("NAV: Crossing Zone. Step 2: Go to Exit (%d, %d)\n",
                      exit_x, exit_y);
        s_pending_final_x = map_x;
        s_pending_final_y = map_y;
        s_has_pending_final = true;

        map_x = exit_x;
        map_y = exit_y;
        uses_via_point = true;
      }
      // 3. If passed Exit, proceed to Final (Fallthrough)
    }
  }

  Serial.printf("NAV: Set Target Map: (%d, %d) %s\n", map_x, map_y,
                uses_via_point ? "[VIA]" : "[DIRECT]");
  ESP_LOGI(TAG, "Set Target Map: (%d, %d)", map_x, map_y);

  // 2. Plan Path
  esp_err_t ret = astar_plan_path((int16_t)start.x, (int16_t)start.y, map_x,
                                  map_y, &g_current_path);

  if (ret != ESP_OK) {
    Serial.printf("NAV ERROR: A* Plan Failed!\n");
    ESP_LOGE(TAG, "A* Plan Failed!");
    // If failed to plan to Via Point, clear pending?
    // Yes, otherwise we might get stuck state.
    s_has_pending_final = false;
    g_nav_status.state = NAV_STATE_ERROR;
    return ESP_FAIL;
  }

  // 3. Smooth Path
  astar_simplify_path(&g_current_path);

  // 3. Update Status
  g_nav_status.target_map_pos.x = map_x;
  g_nav_status.target_map_pos.y = map_y;
  g_nav_status.state = NAV_STATE_NAVIGATING; // Start Moving!
  g_nav_status.path_length = g_current_path.length;
  g_nav_status.current_waypoint = 0; // Reset progress

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
