/**
 * @file tof_sensor.h
 * @brief VL53L0X ToF (Time-of-Flight) Sensor Interface
 * 
 * Manages three VL53L0X ToF sensors via TCA9548A I2C multiplexer
 * - Sensor 0: Left side
 * - Sensor 1: Right side
 * - Sensor 2: Top
 */

#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ToF sensor data structure
 */
typedef struct {
    uint16_t distance_mm;      // Distance in millimeters
    bool valid;                // True if measurement is valid
    uint8_t range_status;      // VL53L0X range status code
} tof_data_t;

/**
 * @brief ToF sensor positions
 */
typedef enum {
    TOF_LEFT = 0,   // Left side sensor (channel 0)
    TOF_RIGHT = 1,  // Right side sensor (channel 1)
    TOF_TOP = 2     // Top sensor (channel 2)
} tof_position_t;

/**
 * @brief Initialize all ToF sensors
 * 
 * Initializes I2C bus and all three VL53L0X sensors via TCA9548A
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t tof_init(void);

/**
 * @brief Read distance from a specific ToF sensor
 * 
 * @param position Sensor position (TOF_LEFT, TOF_RIGHT, or TOF_TOP)
 * @param data Pointer to store measurement data
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t tof_read(tof_position_t position, tof_data_t *data);

/**
 * @brief Read all ToF sensors at once
 * 
 * @param left_data Pointer to store left sensor data
 * @param right_data Pointer to store right sensor data
 * @param top_data Pointer to store top sensor data
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t tof_read_all(tof_data_t *left_data, tof_data_t *right_data, tof_data_t *top_data);

/**
 * @brief Get left side ToF distance
 * 
 * @return Distance in millimeters, or 0xFFFF if invalid
 */
uint16_t tof_get_left_distance(void);

/**
 * @brief Get right side ToF distance
 * 
 * @return Distance in millimeters, or 0xFFFF if invalid
 */
uint16_t tof_get_right_distance(void);

/**
 * @brief Get top ToF distance
 * 
 * @return Distance in millimeters, or 0xFFFF if invalid
 */
uint16_t tof_get_top_distance(void);

#ifdef __cplusplus
}
#endif

#endif // TOF_SENSOR_H

