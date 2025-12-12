/**
 * @file wall_following_v2.h
 * @brief Wall Following V2 - State Machine Header
 */

#ifndef WALL_FOLLOWING_V2_H
#define WALL_FOLLOWING_V2_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// System States
typedef enum { WF2_IDLE, WF2_INIT, WF2_RUNNING, WF2_STOPPED } wf2_state_t;

// Sequential Stages for the Course
typedef enum {
  WF2_STAGE_CONVEX_CORNER_RT,  // 凸角_RT (Start)
  WF2_STAGE_TRAJECTORY_TOP,    // 轨迹_T
  WF2_STAGE_CONVEX_CORNER_LT,  // 凸角_LT
  WF2_STAGE_LONG_EDGE_LEFT,    // 长边_L
  WF2_STAGE_CONVEX_CORNER_LB,  // 凸角_LB
  WF2_STAGE_TRAJECTORY_BOTTOM, // 轨迹_B
  WF2_STAGE_CONVEX_CORNER_RB,  // 凸角_RB
  WF2_STAGE_LONG_EDGE_RIGHT,   // 场边_R
  WF2_STAGE_DONE               // Completed
} wf2_stage_t;

typedef struct {
  wf2_state_t state;
  wf2_stage_t stage;
  float current_x;
  float current_y;
  float current_heading;
  uint16_t tof_front;
  uint16_t tof_right; // Mapped from Left-Front used as Right
  bool is_running;
} wf2_status_t;

esp_err_t wall_following_v2_init(void);
esp_err_t wall_following_v2_start(void);
esp_err_t wall_following_v2_stop(void);
esp_err_t wall_following_v2_get_status(wf2_status_t *status);

#ifdef __cplusplus
}
#endif

#endif // WALL_FOLLOWING_V2_H
