/**
 * @file localization_ekf.h
 * @brief Simple EKF fusion for Encoder (Prediction) + Vive (Correction)
 */

#ifndef LOCALIZATION_EKF_H
#define LOCALIZATION_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>

typedef struct {
  float x;     // Global X (Inches)
  float y;     // Global Y (Inches)
  float theta; // Global Heading (Radians, -PI to PI)
} robot_pose_t;

/**
 * @brief Initialize EKF with a starting pose
 */
void ekf_init(float start_x, float start_y, float start_theta);

/**
 * @brief Prediction Step (High Frequency ~50Hz-100Hz)
 * Call this whenever you read new encoder values.
 *
 * @param d_left   Distance left wheel traveled since last call (inches)
 * @param d_right  Distance right wheel traveled since last call (inches)
 */
void ekf_predict(float d_left, float d_right);

/**
 * @brief Correction Step (Low Frequency ~20Hz)
 * Call this whenever a valid Vive packet is parsed.
 *
 * @param z_x      Vive measured Map X (inches)
 * @param z_y      Vive measured Map Y (inches)
 * @param z_theta  Vive measured Heading (radians). Pass NAN if invalid.
 */
void ekf_update_vive(float z_x, float z_y, float z_theta);

/**
 * @brief Get the current best estimate of the robot pose
 */
robot_pose_t ekf_get_pose(void);

#ifdef __cplusplus
}
#endif

#endif // LOCALIZATION_EKF_H
