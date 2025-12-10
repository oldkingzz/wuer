/**
 * @file tof_sensor.h
 * @brief VL53L0X ToF (Time-of-Flight) Sensor Interface
 *
 * Manages four VL53L0X ToF sensors via TCA9548A I2C multiplexer
 * - Sensor 0 (SD0): Top
 * - Sensor 1 (SD1): Front
 * - Sensor 2 (SD2): Left-Front
 * - Sensor 3 (SD3): Left-Rear
 */

#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ToF sensor configuration
 */
#define TOF_MAX_DISTANCE_MM          500  // Maximum valid distance (mm)
#define TOF_CHANNEL_SWITCH_DELAY_MS  2    // Delay after channel switch (ms)
                                          // 减少到2ms以提高读取速度（只使用2个传感器）
                                          // Reduced to 2ms for faster reading (only using 2 sensors)
                                          // 原来10ms太长，导致50ms周期内无法完成2个传感器读取
                                          // Previous 10ms was too long, couldn't finish 2 sensors in 50ms period

/**
 * @brief ToF sensor data structure
 */
typedef struct {
    uint16_t distance_mm;      // Distance in millimeters
    bool valid;                // True if measurement is valid
    uint8_t range_status;      // VL53L0X range status code
    uint32_t timestamp_ms;     // Timestamp of measurement (millis())
} tof_data_t;

/**
 * @brief ToF sensor positions
 */
typedef enum {
    TOF_TOP = 0,        // Top sensor (SD0, channel 0)
    TOF_FRONT = 1,      // Front sensor (SD1, channel 1)
    TOF_LEFT_FRONT = 2, // Left-Front sensor (SD2, channel 2)
    TOF_LEFT_REAR = 3,  // Left-Rear sensor (SD3, channel 3)

    // 兼容旧代码的别名
    TOF_LEFT = TOF_LEFT_FRONT,  // 兼容旧代码
    TOF_RIGHT = TOF_FRONT       // 兼容旧代码
} tof_position_t;

/**
 * @brief Initialize all ToF sensors
 *
 * Initializes I2C bus and all four VL53L0X sensors via TCA9548A
 *
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t tof_init(void);

/**
 * @brief Set maximum valid distance threshold
 *
 * Measurements beyond this distance will be marked as invalid
 *
 * @param max_distance_mm Maximum distance in millimeters (default: 250mm)
 */
void tof_set_max_distance(uint16_t max_distance_mm);

/**
 * @brief Set channel switch delay
 *
 * Adjust delay after TCA9548A channel switching for speed vs stability tradeoff
 *
 * @param delay_ms Delay in milliseconds (0-10ms, default: 1ms)
 *                 0ms = fastest but may be unstable
 *                 1ms = good balance (default)
 *                 2-5ms = more stable but slower
 */
void tof_set_channel_delay(uint8_t delay_ms);

/**
 * @brief Read distance from a specific ToF sensor
 *
 * @param position Sensor position (TOF_TOP, TOF_FRONT, TOF_LEFT_FRONT, or TOF_LEFT_REAR)
 * @param data Pointer to store measurement data
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t tof_read(tof_position_t position, tof_data_t *data);

/**
 * @brief Backward-compatible helper: read distance by legacy index
 *
 * Legacy index mapping (for old examples such as examples/wall_following.ino):
 *   index 0 -> Front ToF
 *   index 1 -> Left-side ToF
 *   index 2 -> Right-side ToF
 *
 * New code should prefer tof_read()/tof_get_* APIs. This helper exists only
 * to keep legacy example sketches working without modification.
 *
 * @param index 0 = front, 1 = left, 2 = right; other values return 0
 * @return Distance in millimeters, or 0 if invalid/not available
 */
uint16_t tof_read_distance(uint8_t index);

/**
 * @brief Read all ToF sensors at once
 *
 * @param top_data Pointer to store top sensor data
 * @param front_data Pointer to store front sensor data
 * @param left_front_data Pointer to store left-front sensor data
 * @param left_rear_data Pointer to store left-rear sensor data
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t tof_read_all(tof_data_t *top_data, tof_data_t *front_data,
                       tof_data_t *left_front_data, tof_data_t *left_rear_data);

/**
 * @brief Get top ToF distance
 *
 * @return Distance in millimeters, or 0xFFFF if invalid
 */
uint16_t tof_get_top_distance(void);

/**
 * @brief Get front ToF distance
 *
 * @return Distance in millimeters, or 0xFFFF if invalid
 */
uint16_t tof_get_front_distance(void);

/**
 * @brief Get left-front ToF distance
 *
 * @return Distance in millimeters, or 0xFFFF if invalid
 */
uint16_t tof_get_left_front_distance(void);

/**
 * @brief Get left-rear ToF distance
 *
 * @return Distance in millimeters, or 0xFFFF if invalid
 */
uint16_t tof_get_left_rear_distance(void);

// 兼容旧代码的函数别名
#define tof_get_left_distance()  tof_get_left_front_distance()
#define tof_get_right_distance() tof_get_front_distance()

/**
 * @brief Start asynchronous ToF reading task
 *
 * Starts a background FreeRTOS task that continuously reads all ToF sensors.
 * Other tasks can call tof_get_cached_*() functions to get latest readings without blocking.
 *
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t tof_start_async_reading(void);

/**
 * @brief Stop asynchronous ToF reading task
 */
void tof_stop_async_reading(void);

/**
 * @brief Get cached ToF data (non-blocking)
 *
 * Returns the latest cached reading from the async reading task.
 * This function is non-blocking and returns immediately.
 *
 * @param position ToF sensor position
 * @param data Pointer to store cached data
 * @return ESP_OK if data is valid, ESP_FAIL if no valid data available
 */
esp_err_t tof_get_cached(tof_position_t position, tof_data_t *data);

/**
 * @brief Get cached front ToF distance (non-blocking)
 *
 * @return Distance in millimeters, or 0xFFFF if invalid/stale
 */
uint16_t tof_get_cached_front_distance(void);

/**
 * @brief Get cached left-front ToF distance (non-blocking)
 *
 * @return Distance in millimeters, or 0xFFFF if invalid/stale
 */
uint16_t tof_get_cached_left_front_distance(void);

/**
 * @brief Check if cached data is fresh (updated within timeout)
 *
 * @param position ToF sensor position
 * @param timeout_ms Maximum age of data in milliseconds
 * @return true if data is fresh, false if stale or invalid
 */
bool tof_is_data_fresh(tof_position_t position, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // TOF_SENSOR_H

