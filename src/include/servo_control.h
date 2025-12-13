/**
 * @file servo_control.h
 * @brief Servo Control Interface for MG996R
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize the servo on Pin 9
 * @return ESP_OK or ESP_FAIL
 */
esp_err_t servo_init(void);

/**
 * @brief Set the servo angle
 * @param angle Angle in degrees (0-180)
 * @return ESP_OK or ESP_FAIL
 */
esp_err_t servo_set_angle(int angle);

/**
 * @brief Sweep servo from 0 to 60 degrees and back
 * Useful for testing or mechanism actuation
 */
void servo_sweep_0_60(void);

/**
 * @brief Stop the servo sweep
 */
void servo_stop_sweep(void);

#endif // SERVO_CONTROL_H
