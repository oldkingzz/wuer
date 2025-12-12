/**
 * @file vive_sensor.h
 * @brief Vive Positioning System Interface
 * 
 * Manages two Vive sensors for robot positioning
 */

#ifndef VIVE_SENSOR_H
#define VIVE_SENSOR_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Vive sensor position data
 */
typedef struct {
    uint16_t x;           // X coordinate (0-8191)
    uint16_t y;           // Y coordinate (0-8191)
    bool valid;           // True if receiving valid signal
    uint8_t status;       // VIVE_NO_SIGNAL, VIVE_SYNC_ONLY, or VIVE_RECEIVING
} vive_data_t;

/**
 * @brief Vive sensor position identifier
 */
typedef enum {
    VIVE_SENSOR_1 = 0,    // First Vive sensor
    VIVE_SENSOR_2 = 1     // Second Vive sensor
} vive_sensor_id_t;

/**
 * @brief Initialize both Vive sensors
 * 
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vive_init(void);

/**
 * @brief Read data from a specific Vive sensor
 * 
 * @param sensor_id Which sensor to read (VIVE_SENSOR_1 or VIVE_SENSOR_2)
 * @param data Pointer to store the sensor data
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vive_read(vive_sensor_id_t sensor_id, vive_data_t *data);

/**
 * @brief Read data from both Vive sensors
 * 
 * @param sensor1_data Pointer to store sensor 1 data
 * @param sensor2_data Pointer to store sensor 2 data
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vive_read_all(vive_data_t *sensor1_data, vive_data_t *sensor2_data);

/**
 * @brief Start asynchronous Vive reading task
 *
 * 在后台以固定频率读取两个Vive传感器，并更新内部缓存数据。
 * 其他模块（导航、状态监控）应优先使用缓存接口而不是直接调用 vive_read_all()。
 *
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t vive_start_async_reading(void);

/**
 * @brief Stop asynchronous Vive reading task
 */
void vive_stop_async_reading(void);

/**
 * @brief Get X coordinate from sensor 1
 * 
 * @return X coordinate (0-8191), or 0 if invalid
 */
uint16_t vive_get_sensor1_x(void);

/**
 * @brief Get Y coordinate from sensor 1
 * 
 * @return Y coordinate (0-8191), or 0 if invalid
 */
uint16_t vive_get_sensor1_y(void);

/**
 * @brief Get X coordinate from sensor 2
 * 
 * @return X coordinate (0-8191), or 0 if invalid
 */
uint16_t vive_get_sensor2_x(void);

/**
 * @brief Get Y coordinate from sensor 2
 * 
 * @return Y coordinate (0-8191), or 0 if invalid
 */
uint16_t vive_get_sensor2_y(void);

/**
 * @brief Check if sensor 1 is receiving valid signal
 * 
 * @return true if receiving, false otherwise
 */
bool vive_sensor1_is_valid(void);

/**
 * @brief Check if sensor 2 is receiving valid signal
 * 
 * @return true if receiving, false otherwise
 */
bool vive_sensor2_is_valid(void);

/**
 * @brief Get latest cached data for a specific Vive sensor (non-blocking)
 *
 * 从后台任务更新的缓存中读取最新的一帧数据，不会再次访问硬件。
 */
esp_err_t vive_get_latest(vive_sensor_id_t sensor_id, vive_data_t *data);

/**
 * @brief Get latest cached data for both Vive sensors (non-blocking)
 *
 * 从缓存中一次性拷贝两个Vive传感器的数据，供导航等模块使用。
 */
esp_err_t vive_get_latest_all(vive_data_t *sensor1_data, vive_data_t *sensor2_data);

#ifdef __cplusplus
}
#endif

#endif // VIVE_SENSOR_H

