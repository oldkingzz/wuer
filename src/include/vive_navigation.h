/**
 * @file vive_navigation.h
 * @brief A* path planning navigation system using Vive localization and a grid map.
 */

#ifndef VIVE_NAVIGATION_H
#define VIVE_NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "astar.h"
#include "grid_map.h"
#include "nav_config.h"   // Provides NAV_* / NAV_MISSION_* constants and nav_goal_t definitions

/* ========== Data structures ========== */

// Vive coordinate point (0–8191 in both axes)
typedef struct {
    uint16_t x;
    uint16_t y;
} vive_point_t;

// Robot pose in Vive coordinates
typedef struct {
    uint16_t x;       // Vive X position
    uint16_t y;       // Vive Y position
    float    heading; // Heading in degrees (0–360)
    bool     valid;   // Whether pose is valid
} vive_pose_t;

// Low-level navigation state (A* + path following)
typedef enum {
    NAV_STATE_IDLE = 0,      // Idle
    NAV_STATE_PLANNING,      // Planning path
    NAV_STATE_NAVIGATING,    // Following path
    NAV_STATE_ARRIVED,       // Reached target
    NAV_STATE_ERROR          // Error
} nav_state_t;

// Navigation status information
typedef struct {
    nav_state_t  state;             // Current state
    vive_pose_t  current_pose;      // Current pose in Vive coordinates
    vive_point_t target;            // Target in Vive coordinates
    map_point_t  current_map_pos;   // Current map position (pixels)
    map_point_t  target_map_pos;    // Target map position (pixels)
    uint16_t     path_length;       // Total path length (number of waypoints)
    uint16_t     current_waypoint;  // Index of current waypoint
    float        distance_to_target;// Distance to target (pixels)
    float        heading_error;     // Heading error (degrees)
    float        linear_velocity;   // Current linear velocity (m/s)
    float        angular_velocity;  // Current angular velocity (rad/s)
} nav_status_t;

// High-level mission state machine for "pre-point → align → impact → return"
typedef enum {
    NAV_MISSION_STATE_IDLE = 0,        // No mission running
    NAV_MISSION_STATE_GOTO_PRE_POINT,  // Use Vive + A* to go to pre-point
    NAV_MISSION_STATE_ALIGN_TOF,       // Align using ToF (rotate in place)
    NAV_MISSION_STATE_FORWARD_IMPACT,  // Drive forward to impact button/target
    NAV_MISSION_STATE_RETURN,          // Drive backward to a safe position
    NAV_MISSION_STATE_DONE,            // Mission finished successfully
    NAV_MISSION_STATE_ERROR            // Mission aborted / failed
} nav_mission_state_t;

// Mission status information (for higher layers / web UI)
typedef struct {
    nav_mission_state_t state;          // Current mission state
    nav_goal_id_t       goal_id;        // Which goal from nav_config.h is being used
    nav_state_t         nav_state;      // Underlying low-level navigation state
    uint32_t            state_elapsed_ms; // Time in current mission state (ms)
} nav_mission_status_t;

/* ========== Low-level navigation API (Vive + A*) ========== */

// Initialize Vive navigation system
esp_err_t vive_nav_init(void);

// Set target in Vive coordinates (0–8191) and plan path
esp_err_t vive_nav_set_target(uint16_t target_x, uint16_t target_y);

// Set target in map coordinates (pixels) and plan path
esp_err_t vive_nav_set_target_map(int16_t map_x, int16_t map_y);

// Convenience helper: set target using a pre-defined goal id from nav_config.h
esp_err_t vive_nav_set_target_goal(nav_goal_id_t goal_id);

// Start navigation towards previously set target
esp_err_t vive_nav_start(void);

// Stop navigation
esp_err_t vive_nav_stop(void);

// Get current low-level navigation status
esp_err_t vive_nav_get_status(nav_status_t *status);

// Get current robot pose (Vive coordinates)
esp_err_t vive_nav_get_pose(vive_pose_t *pose);

// Get current planned path
esp_err_t vive_nav_get_path(path_t *path);

// Force path replanning from current pose to current target
esp_err_t vive_nav_replan(void);

/* ========== High-level mission API (pre-point → align → impact → return) ========== */

// Start a high-level impact mission for the given pre-defined goal id.
esp_err_t nav_mission_start(nav_goal_id_t goal_id);

// Immediately stop the current mission (if any) and stop the chassis.
esp_err_t nav_mission_stop(void);

// Get current mission state and basic timing information.
esp_err_t nav_mission_get_status(nav_mission_status_t *status);

#endif // VIVE_NAVIGATION_H
