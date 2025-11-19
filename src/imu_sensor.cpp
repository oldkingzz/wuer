/**
 * @file imu_sensor.cpp
 * @brief MPU6050 IMU Sensor Implementation
 */

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "include/imu_sensor.h"
#include "include/gpio_config.h"
#include "esp_log.h"

static const char *TAG = "IMU_SENSOR";

// MPU6050 object
static Adafruit_MPU6050 mpu;

// Initialization status
static bool imu_initialized = false;

// Latest measurement
static imu_data_t g_imu_data = {0};

// Mutex for thread safety
static SemaphoreHandle_t imu_mutex = NULL;

/**
 * @brief Select TCA9548A channel
 */
static void tca_select(uint8_t channel)
{
    if (channel > 7) return;
    
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

/**
 * @brief Initialize IMU sensor
 */
esp_err_t imu_init(void)
{
    ESP_LOGI(TAG, "Initializing IMU sensor...");
    Serial.print("Init MPU6050 (Channel 3)... ");

    // Create mutex
    imu_mutex = xSemaphoreCreateMutex();
    if (imu_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Select IMU channel
    tca_select(IMU_CHANNEL);
    delay(10);

    // Initialize MPU6050
    if (!mpu.begin()) {
        ESP_LOGE(TAG, "Failed to init MPU6050");
        Serial.println("Failed!");
        imu_initialized = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6050 initialized");
    Serial.println("Success!");

    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    ESP_LOGI(TAG, "MPU6050 configured: Accel=8G, Gyro=500deg/s, Filter=21Hz");
    Serial.println("MPU6050 configured successfully");

    imu_initialized = true;
    return ESP_OK;
}

/**
 * @brief Read complete IMU data
 */
esp_err_t imu_read(imu_data_t *data)
{
    if (data == NULL) return ESP_FAIL;
    if (imu_mutex == NULL) return ESP_FAIL;

    xSemaphoreTake(imu_mutex, portMAX_DELAY);

    // Check if IMU is initialized
    if (!imu_initialized) {
        data->accel.x = data->accel.y = data->accel.z = 0.0f;
        data->gyro.x = data->gyro.y = data->gyro.z = 0.0f;
        data->temperature = 0.0f;
        data->valid = false;
        xSemaphoreGive(imu_mutex);
        return ESP_FAIL;
    }

    // Select IMU channel
    tca_select(IMU_CHANNEL);
    delay(1);

    // Read sensor data
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Store data
    data->accel.x = accel.acceleration.x;
    data->accel.y = accel.acceleration.y;
    data->accel.z = accel.acceleration.z;

    data->gyro.x = gyro.gyro.x;
    data->gyro.y = gyro.gyro.y;
    data->gyro.z = gyro.gyro.z;

    data->temperature = temp.temperature;
    data->valid = true;

    // Update global data
    g_imu_data = *data;

    xSemaphoreGive(imu_mutex);
    return ESP_OK;
}

/**
 * @brief Read only accelerometer data
 */
esp_err_t imu_read_accel(imu_accel_t *accel)
{
    imu_data_t data;
    esp_err_t ret = imu_read(&data);
    if (ret == ESP_OK && accel != NULL) {
        *accel = data.accel;
    }
    return ret;
}

/**
 * @brief Read only gyroscope data
 */
esp_err_t imu_read_gyro(imu_gyro_t *gyro)
{
    imu_data_t data;
    esp_err_t ret = imu_read(&data);
    if (ret == ESP_OK && gyro != NULL) {
        *gyro = data.gyro;
    }
    return ret;
}

/**
 * @brief Get Z-axis gyroscope value
 */
float imu_get_gyro_z(void)
{
    return g_imu_data.valid ? g_imu_data.gyro.z : 0.0f;
}

/**
 * @brief Get temperature from IMU
 */
float imu_get_temperature(void)
{
    return g_imu_data.valid ? g_imu_data.temperature : 0.0f;
}

