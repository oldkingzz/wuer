/**
 * @file tof_sensor.cpp
 * @brief VL53L0X ToF Sensor Implementation
 */

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "include/tof_sensor.h"
#include "include/gpio_config.h"
#include "esp_log.h"

static const char *TAG = "TOF_SENSOR";

// Sensor objects
static Adafruit_VL53L0X lox_left;
static Adafruit_VL53L0X lox_right;
static Adafruit_VL53L0X lox_top;

// Initialization status
static bool tof_left_initialized = false;
static bool tof_right_initialized = false;
static bool tof_top_initialized = false;

// Latest measurements
static tof_data_t g_left_data = {0, false, 0};
static tof_data_t g_right_data = {0, false, 0};
static tof_data_t g_top_data = {0, false, 0};

// Mutex for thread safety
static SemaphoreHandle_t tof_mutex = NULL;

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
 * @brief Initialize all ToF sensors
 */
esp_err_t tof_init(void)
{
    ESP_LOGI(TAG, "Initializing ToF sensors...");

    // Create mutex
    tof_mutex = xSemaphoreCreateMutex();
    if (tof_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        Serial.println("ERROR: Failed to create ToF mutex");
        return ESP_FAIL;
    }

#if USE_TCA9548A
    // 使用TCA9548A模式
    Serial.println("Initializing ToF sensors via TCA9548A...");
    Serial.flush();

    // I2C is already initialized in Wire.begin() from main
    // Just verify we can communicate with TCA9548A
    Serial.print("Checking TCA9548A at address 0x");
    Serial.print(TCA9548A_ADDR, HEX);
    Serial.print("... ");
    Serial.flush();

    Wire.beginTransmission(TCA9548A_ADDR);
    uint8_t error = Wire.endTransmission();

    if (error != 0) {
        ESP_LOGE(TAG, "TCA9548A not found! Error: %d", error);
        Serial.print("Not found! (Error ");
        Serial.print(error);
        Serial.println(")");
        Serial.println("Skipping ToF initialization");
        Serial.flush();
        return ESP_FAIL;
    }

    Serial.println("Found!");
    Serial.flush();
#else
    // 直连模式（不使用TCA9548A）
    Serial.println("Initializing ToF sensors (direct connection, no TCA9548A)...");
    Serial.flush();
#endif

    // Initialize left ToF sensor
#if USE_TCA9548A
    tca_select(TOF_LEFT_CHANNEL);
    delay(10);
    Serial.print("Init ToF Left (Channel 0)... ");
#else
    Serial.print("Init ToF Left (Direct, 0x29)... ");
#endif
    if (!lox_left.begin()) {
        ESP_LOGE(TAG, "Failed to init left ToF");
        Serial.println("Failed!");
        tof_left_initialized = false;
    } else {
        ESP_LOGI(TAG, "Left ToF initialized");
        Serial.println("Success!");
        tof_left_initialized = true;
    }

#if USE_TCA9548A
    // Initialize right ToF sensor (channel 1) - 仅在使用TCA9548A时
    tca_select(TOF_RIGHT_CHANNEL);
    delay(10);
    Serial.print("Init ToF Right (Channel 1)... ");
    if (!lox_right.begin()) {
        ESP_LOGE(TAG, "Failed to init right ToF");
        Serial.println("Failed!");
        tof_right_initialized = false;
    } else {
        ESP_LOGI(TAG, "Right ToF initialized");
        Serial.println("Success!");
        tof_right_initialized = true;
    }

    // Initialize top ToF sensor (channel 2) - 仅在使用TCA9548A时
    tca_select(TOF_TOP_CHANNEL);
    delay(10);
    Serial.print("Init ToF Top (Channel 2)... ");
    if (!lox_top.begin()) {
        ESP_LOGE(TAG, "Failed to init top ToF");
        Serial.println("Failed!");
        tof_top_initialized = false;
    } else {
        ESP_LOGI(TAG, "Top ToF initialized");
        Serial.println("Success!");
        tof_top_initialized = true;
    }
#else
    // 直连模式下，只能使用一个VL53L0X（地址0x29）
    Serial.println("Note: Only one VL53L0X supported in direct mode");
    tof_right_initialized = false;
    tof_top_initialized = false;
#endif

    ESP_LOGI(TAG, "ToF sensors initialization complete");
    return ESP_OK;
}

/**
 * @brief Read distance from a specific ToF sensor
 */
esp_err_t tof_read(tof_position_t position, tof_data_t *data)
{
    if (data == NULL) return ESP_FAIL;
    if (tof_mutex == NULL) return ESP_FAIL;

    xSemaphoreTake(tof_mutex, portMAX_DELAY);

    VL53L0X_RangingMeasurementData_t measure;
    Adafruit_VL53L0X *sensor = NULL;
    uint8_t channel = 0;
    bool initialized = false;

    // Select sensor and channel
    switch (position) {
        case TOF_LEFT:
            sensor = &lox_left;
            channel = TOF_LEFT_CHANNEL;
            initialized = tof_left_initialized;
            break;
        case TOF_RIGHT:
            sensor = &lox_right;
            channel = TOF_RIGHT_CHANNEL;
            initialized = tof_right_initialized;
            break;
        case TOF_TOP:
            sensor = &lox_top;
            channel = TOF_TOP_CHANNEL;
            initialized = tof_top_initialized;
            break;
        default:
            xSemaphoreGive(tof_mutex);
            return ESP_FAIL;
    }

    // Check if sensor is initialized
    if (!initialized) {
        data->distance_mm = 0;
        data->range_status = 255;
        data->valid = false;
        xSemaphoreGive(tof_mutex);
        return ESP_FAIL;
    }

    // Select channel and read
#if USE_TCA9548A
    tca_select(channel);
    delay(1);
#endif
    sensor->rangingTest(&measure, false);

    // Store data
    data->distance_mm = measure.RangeMilliMeter;
    data->range_status = measure.RangeStatus;
    data->valid = (measure.RangeStatus != 4);  // 4 = out of range

    xSemaphoreGive(tof_mutex);
    return ESP_OK;
}

/**
 * @brief Read all ToF sensors at once
 */
esp_err_t tof_read_all(tof_data_t *left_data, tof_data_t *right_data, tof_data_t *top_data)
{
    esp_err_t ret;

    ret = tof_read(TOF_LEFT, left_data);
    if (ret == ESP_OK) {
        g_left_data = *left_data;
    }

    ret = tof_read(TOF_RIGHT, right_data);
    if (ret == ESP_OK) {
        g_right_data = *right_data;
    }

    ret = tof_read(TOF_TOP, top_data);
    if (ret == ESP_OK) {
        g_top_data = *top_data;
    }

    return ESP_OK;
}

/**
 * @brief Get left side ToF distance
 */
uint16_t tof_get_left_distance(void)
{
    return g_left_data.valid ? g_left_data.distance_mm : 0xFFFF;
}

/**
 * @brief Get right side ToF distance
 */
uint16_t tof_get_right_distance(void)
{
    return g_right_data.valid ? g_right_data.distance_mm : 0xFFFF;
}

/**
 * @brief Get top ToF distance
 */
uint16_t tof_get_top_distance(void)
{
    return g_top_data.valid ? g_top_data.distance_mm : 0xFFFF;
}


