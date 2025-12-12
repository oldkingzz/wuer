/**
 * @file nav_mission.h
 * @brief Mission Control State Machine Logic
 *
 * Handles high-level complex missions (High Tower, Low Tower, Nexus)
 * Dispatches to specific logic based on Goal ID.
 */

#ifndef NAV_MISSION_H
#define NAV_MISSION_H

#include "esp_err.h"
#include "nav_config.h"
#include "vive_navigation.h" // Access to underlying nav_mission_status_t

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the mission control system (Create task)
esp_err_t nav_mission_init(void);

// External wrapper functions (mapped to vive_navigation.cpp's stubs ideally)
// But since we are implementing the logic here, we provide the real
// implementation. Note: vive_navigation.cpp has stubs for these. We should
// likely remove those stubs or make them call these functions. For now, let's
// define the core logic functions here.

// The main entry points that vive_navigation.cpp (or web server) should call:
esp_err_t mission_start(nav_goal_id_t goal_id);
esp_err_t mission_stop(void);
esp_err_t mission_get_status(nav_mission_status_t *status);

#ifdef __cplusplus
}
#endif

#endif // NAV_MISSION_H
