/**
 * @file localization_ekf.cpp
 * @brief EKF Implementation for Differential Drive Robot
 *
 * State: [x, y, theta]
 * Control: [d_left, d_right] (Encoder delta distances)
 * Measurement: [x_vive, y_vive, theta_vive]
 */

#include "include/localization_ekf.h"
#include "include/chassis_v2.h" // For CHASSIS_V2_WHEEL_BASE_M
#include <math.h>
#include <stdio.h> // For NAN check if needed

// ==========================================
// Parameters
// ==========================================

// Robot physical parameters
// Convert Meters to Inches (1 m = 39.37 inches)
#define TRACK_WIDTH (CHASSIS_V2_WHEEL_BASE_M * 39.37f)

// Process Noise (How much we trust our encoder model)
// Q matrix diagonal elements
#define Q_X 0.001f     // Uncertainty in X per step
#define Q_Y 0.001f     // Uncertainty in Y per step
#define Q_THETA 0.001f // MORE TRUST in Encoder (was 0.002)

// Measurement Noise (How much we trust Vive)
// R matrix diagonal elements
// Increased R_X/R_Y to 2.0 (was 0.5) to filter out Vive position jitter at
// edges
#define R_X 2.0f
#define R_Y 2.0f
#define R_THETA                                                                \
  1.5f // LESS TRUST in Vive Heading (was 0.1). High value masks jumps.

// ==========================================
// State Variables
// ==========================================

static robot_pose_t g_x = {0}; // State Vector estimate
static float g_P[3][3] = {0};  // Covariance Matrix estimate

// Helper: Normalize angle to [-PI, PI]
static float normalize_angle(float angle) {
  while (angle > M_PI)
    angle -= 2.0f * M_PI;
  while (angle < -M_PI)
    angle += 2.0f * M_PI;
  return angle;
}

// 1. Initialization
void ekf_init(float start_x, float start_y, float start_theta) {
  g_x.x = start_x;
  g_x.y = start_y;
  g_x.theta = normalize_angle(start_theta);

  // Initialize Covariance P with some uncertainty
  g_P[0][0] = 1.0f;
  g_P[0][1] = 0.0f;
  g_P[0][2] = 0.0f;
  g_P[1][0] = 0.0f;
  g_P[1][1] = 1.0f;
  g_P[1][2] = 0.0f;
  g_P[2][0] = 0.0f;
  g_P[2][1] = 0.0f;
  g_P[2][2] = 0.5f;
}

// 2. Predict Step (Encoder Dead Reckoning)
// Uses standard differential drive approximations
void ekf_predict(float d_left, float d_right) {
  // A. Control Input
  // A. Control Input
  float d_center = (d_left + d_right) / 2.0f;
  // Revert: Standard differential drive formula (Right - Left) / Width.
  // With ENCODER2_REVERSED=true, d_left is positive for forward motion.
  // Turning Left (CCW) -> Right > Left -> d_theta > 0. Correct.
  float d_theta = (d_right - d_left) / TRACK_WIDTH;

  // B. State Transition (Jacobian F is approximated as Identity for covariance
  // update in simple cases, but strictly it depends on theta. Here we keep P
  // update simple for MCU efficiency)

  // Update State
  float old_theta = g_x.theta;
  float new_theta = normalize_angle(old_theta + d_theta);

  // Runge-Kutta 2nd order integration for position (or just average theta)
  float avg_theta = normalize_angle(old_theta + d_theta / 2.0f);

  g_x.x += d_center * cosf(avg_theta);
  g_x.y += d_center * sinf(avg_theta);
  g_x.theta = new_theta;

  // C. Covariance Update: P = F*P*F' + Q
  // For small d_theta, F is close to Identity. We simply add Process Noise Q.
  // Ideally calculate Jacobian F = [1 0 -ds*sin; 0 1 ds*cos; 0 0 1].
  // Simplified:
  g_P[0][0] += Q_X;
  g_P[1][1] += Q_Y;
  g_P[2][2] += Q_THETA;

  // (Optional) Add cross-terms if doing full Matrix multiplication,
  // but independent noise addition is often sufficient for mobile robots.
}

// 3. Update Step (Vive Correction)
void ekf_update_vive(float z_x, float z_y, float z_theta) {
  // We treat each measurement (X, Y, Theta) independently to simplify matrix
  // inversion to scalar division (Inverse Variance weighting). This is valid if
  // measurement noises are uncorrelated.

  // --- Update X ---
  float S_x = g_P[0][0] + R_X;          // Innovation covariance
  float K_x = g_P[0][0] / S_x;          // Kalman Gain
  float y_x = z_x - g_x.x;              // Innovation (Measurement Residual)
  g_x.x += K_x * y_x;                   // Update State
  g_P[0][0] = (1.0f - K_x) * g_P[0][0]; // Update Covariance

  // --- Update Y ---
  float S_y = g_P[1][1] + R_Y;
  float K_y = g_P[1][1] / S_y;
  float y_y = z_y - g_x.y;
  g_x.y += K_y * y_y;
  g_P[1][1] = (1.0f - K_y) * g_P[1][1];

  // --- Update Theta (if valid) ---
  if (!isnan(z_theta)) {
    float S_t = g_P[2][2] + R_THETA;
    float K_t = g_P[2][2] / S_t;
    float y_t = normalize_angle(z_theta - g_x.theta); // Important: Angle diff!
    g_x.theta = normalize_angle(g_x.theta + K_t * y_t);
    g_P[2][2] = (1.0f - K_t) * g_P[2][2];
  }
}

// 4. Getter
robot_pose_t ekf_get_pose(void) { return g_x; }
