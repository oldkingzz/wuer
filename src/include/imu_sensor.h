/**
 * @file imu_sensor.h
 * @brief MPU6050 IMU (Inertial Measurement Unit) Sensor Interface
 * 
 * Manages MPU6050 6-axis IMU sensor via TCA9548A I2C multiplexer
 * Provides accelerometer and gyroscope data
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IMU accelerometer data structure
 */
typedef struct {
    float x;  // X-axis acceleration (m/s^2)
    float y;  // Y-axis acceleration (m/s^2)
    float z;  // Z-axis acceleration (m/s^2)
} imu_accel_t;

/**
 * @brief IMU gyroscope data structure
 */
typedef struct {
    float x;  // X-axis angular velocity (rad/s)
    float y;  // Y-axis angular velocity (rad/s)
    float z;  // Z-axis angular velocity (rad/s)
} imu_gyro_t;

/**
 * @brief Complete IMU data structure
 */
typedef struct {
    imu_accel_t accel;     // Accelerometer data
    imu_gyro_t gyro;       // Gyroscope data
    float temperature;     // Temperature in Celsius
    bool valid;            // True if data is valid
} imu_data_t;

/**
 * @brief Initialize IMU sensor
 * 
 * Initializes MPU6050 sensor via TCA9548A channel 3
 * Configures accelerometer range, gyro range, and filter bandwidth
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t imu_init(void);

/**
 * @brief Read complete IMU data
 * 
 * @param data Pointer to store IMU data
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t imu_read(imu_data_t *data);

/**
 * @brief Read only accelerometer data
 * 
 * @param accel Pointer to store accelerometer data
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t imu_read_accel(imu_accel_t *accel);

/**
 * @brief Read only gyroscope data
 * 
 * @param gyro Pointer to store gyroscope data
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t imu_read_gyro(imu_gyro_t *gyro);

/**
 * @brief Get Z-axis gyroscope value (for rotation detection)
 * 
 * @return Z-axis angular velocity in rad/s
 */
float imu_get_gyro_z(void);

/**
 * @brief Get temperature from IMU
 * 
 * @return Temperature in Celsius
 */
float imu_get_temperature(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_SENSOR_H

