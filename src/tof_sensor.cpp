/**
 * @file tof_sensor.cpp
 * @brief VL53L0X ToF Sensor Implementation
 */

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "include/tof_sensor.h"
#include "include/gpio_config.h"
#include "include/i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "TOF_SENSOR";

// Sensor objects
static Adafruit_VL53L0X lox_top;         // SD0
static Adafruit_VL53L0X lox_front;       // SD1
static Adafruit_VL53L0X lox_left_front;  // SD2
static Adafruit_VL53L0X lox_left_rear;   // SD3

// Initialization status
static bool tof_top_initialized = false;
static bool tof_front_initialized = false;
static bool tof_left_front_initialized = false;
static bool tof_left_rear_initialized = false;

// Latest measurements (cached data for async reading)
static tof_data_t g_top_data = {0, false, 0, 0};
static tof_data_t g_front_data = {0, false, 0, 0};
static tof_data_t g_left_front_data = {0, false, 0, 0};
static tof_data_t g_left_rear_data = {0, false, 0, 0};

// Mutex for thread safety
static SemaphoreHandle_t tof_mutex = NULL;

// Maximum valid distance threshold (configurable)
static uint16_t g_max_distance_mm = TOF_MAX_DISTANCE_MM;

// Channel switch delay (configurable)
static uint8_t g_channel_delay_ms = TOF_CHANNEL_SWITCH_DELAY_MS;

// Async reading task handle
static TaskHandle_t tof_async_task_handle = NULL;
static bool tof_async_running = false;

/**
 * @brief Select TCA9548A channel (simple version for initialization)
 *
 * 简单版本，只写入通道选择，不验证。
 * 用于初始化阶段，因为此时传感器还没有 begin()，验证可能会失败。
 *
 * Simple version that only writes channel selection without verification.
 * Used during initialization when sensors haven't been begin() yet.
 */
static void tca_select(uint8_t channel)
{
    if (channel > 7) return;

    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();

    // 给通道切换一点时间稳定
    // Give the channel switch some time to settle
    delayMicroseconds(100);
}

/**
 * @brief Robustly check that TCA9548A is present on the bus
 *
 * 有些时候上电时TCA9548A还没完全稳定，第一次访问会失败，这里做几次重试，
 * 这样就不会因为偶发的I2C错误导致整套ToF初始化直接失败。
 */
static bool tca_check_ready(uint8_t attempts, uint16_t delay_ms)
{
    for (uint8_t i = 0; i < attempts; ++i) {
        Wire.beginTransmission(TCA9548A_ADDR);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            return true;
        }

        ESP_LOGW(TAG, "TCA9548A not responding (attempt %d/%d, error=%d)",
                 (int)(i + 1), (int)attempts, (int)error);
        Serial.print("  TCA9548A not responding, attempt ");
        Serial.print(i + 1);
        Serial.print("/");
        Serial.print(attempts);
        Serial.print(", error=");
        Serial.println(error);
        Serial.flush();

        if (delay_ms > 0) {
            delay(delay_ms);
        }
    }

    return false;
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

    // Scan I2C bus first to help stabilize communication
    Serial.println("Scanning I2C bus...");
    Serial.flush();
    int deviceCount = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("  Found device: 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            deviceCount++;
        }
    }
    Serial.print("Total devices found: ");
    Serial.println(deviceCount);
    Serial.flush();

    // I2C is already initialized in Wire.begin() from main
    // Just verify we can communicate with TCA9548A (with retries)
    Serial.print("Checking TCA9548A at address 0x");
    Serial.print(TCA9548A_ADDR, HEX);
    Serial.println(" (with retries)...");
    Serial.flush();

    // 尝试多次，以避免上电瞬间总线不稳定导致的一次性失败
    if (!tca_check_ready(3, 100)) {
        ESP_LOGE(TAG, "TCA9548A not found after retries");
        Serial.println("Not found after retries – skipping ToF initialization");
        Serial.flush();
        return ESP_FAIL;
    }

    Serial.println("TCA9548A found");
    Serial.flush();
#else
    // 直连模式（不使用TCA9548A）
    Serial.println("Initializing ToF sensors (direct connection, no TCA9548A)...");
    Serial.flush();
#endif

    // Initialize Top ToF sensor (SD0)
#if USE_TCA9548A
    tca_select(TOF_TOP_CHANNEL);
    delay(10);  // 只需要10ms，和test_tca.ino一致
    Serial.print("Init ToF Top (SD0, Channel 0)... ");
    Serial.flush();
#else
    Serial.print("Init ToF Top (Direct, 0x29)... ");
#endif
    // 重要：只尝试一次！VL53L0X的begin()内部会调用Wire.begin()，
    // 重复调用会重置I2C总线，导致TCA9548A通道选择被清空
    if (lox_top.begin()) {
        ESP_LOGI(TAG, "Top ToF initialized");
        Serial.println("Success!");
        Serial.flush();
        tof_top_initialized = true;
    } else {
        ESP_LOGE(TAG, "Failed to init top ToF");
        Serial.println("Failed!");
        Serial.flush();
        tof_top_initialized = false;
    }

#if USE_TCA9548A
    // Initialize Front ToF sensor (SD1)
    tca_select(TOF_FRONT_CHANNEL);
    delay(10);
    Serial.print("Init ToF Front (SD1, Channel 1)... ");
    Serial.flush();
    if (lox_front.begin()) {
        ESP_LOGI(TAG, "Front ToF initialized");
        Serial.println("Success!");
        Serial.flush();

        // 使用默认测距时间33ms（更稳定）
        // 如果需要更快，可以设置为20ms: setMeasurementTimingBudgetMicroSeconds(20000)
        Serial.println("  Using default timing budget (33ms, more stable)");
        Serial.flush();

        tof_front_initialized = true;
    } else {
        ESP_LOGE(TAG, "Failed to init front ToF");
        Serial.println("Failed!");
        Serial.flush();
        tof_front_initialized = false;
    }

    // Initialize Left-Front ToF sensor (SD2)
    tca_select(TOF_LEFT_FRONT_CHANNEL);
    delay(10);
    Serial.print("Init ToF Left-Front (SD2, Channel 2)... ");
    Serial.flush();
    if (lox_left_front.begin()) {
        ESP_LOGI(TAG, "Left-Front ToF initialized");
        Serial.println("Success!");
        Serial.flush();

        // 使用默认测距时间33ms（更稳定）
        // 如果需要更快，可以设置为20ms: setMeasurementTimingBudgetMicroSeconds(20000)
        Serial.println("  Using default timing budget (33ms, more stable)");
        Serial.flush();

        tof_left_front_initialized = true;
    } else {
        ESP_LOGE(TAG, "Failed to init left-front ToF");
        Serial.println("Failed!");
        Serial.flush();
        tof_left_front_initialized = false;
    }

    // Initialize Left-Rear ToF sensor (SD3)
    tca_select(TOF_LEFT_REAR_CHANNEL);
    delay(10);
    Serial.print("Init ToF Left-Rear (SD3, Channel 3)... ");
    Serial.flush();
    if (lox_left_rear.begin()) {
        ESP_LOGI(TAG, "Left-Rear ToF initialized");
        Serial.println("Success!");
        Serial.flush();
        tof_left_rear_initialized = true;
    } else {
        ESP_LOGE(TAG, "Failed to init left-rear ToF");
        Serial.println("Failed!");
        Serial.flush();
        tof_left_rear_initialized = false;
    }
#else
    // 直连模式下，只能使用一个VL53L0X（地址0x29）
    Serial.println("Note: Only one VL53L0X supported in direct mode");
    tof_front_initialized = false;
    tof_left_front_initialized = false;
    tof_left_rear_initialized = false;
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
        case TOF_TOP:
            sensor = &lox_top;
            channel = TOF_TOP_CHANNEL;
            initialized = tof_top_initialized;
            break;
        case TOF_FRONT:
            sensor = &lox_front;
            channel = TOF_FRONT_CHANNEL;
            initialized = tof_front_initialized;
            break;
        case TOF_LEFT_FRONT:
            sensor = &lox_left_front;
            channel = TOF_LEFT_FRONT_CHANNEL;
            initialized = tof_left_front_initialized;
            break;
        case TOF_LEFT_REAR:
            sensor = &lox_left_rear;
            channel = TOF_LEFT_REAR_CHANNEL;
            initialized = tof_left_rear_initialized;
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

    // Select channel and read (protect shared I2C/TCA bus)
#if USE_TCA9548A
    i2c_bus_lock();
#endif

    // Select channel and read
#if USE_TCA9548A
    tca_select(channel);
    if (g_channel_delay_ms > 0) {
        delay(g_channel_delay_ms);
    }
#endif

    // 重试机制：如果读取失败，在当前通道重试最多3次
    // Retry mechanism: if read fails, retry up to 3 times on current channel
    const uint8_t max_retries = 3;
    bool success = false;

    for (uint8_t retry = 0; retry < max_retries; retry++) {
        sensor->rangingTest(&measure, false);

        // 检查是否读取成功
        // Check if read was successful
        if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 65535) {
            // 成功读取
            success = true;
            break;
        }

        // 读取失败，等待一小段时间后重试
        // Read failed, wait a bit before retry
        if (retry < max_retries - 1) {
            ESP_LOGW(TAG, "ToF channel %d read failed (status=%d, dist=%d), retry %d/%d",
                     channel, measure.RangeStatus, measure.RangeMilliMeter, retry + 1, max_retries);
            delay(10);  // 等待10ms后重试
        }
    }

    if (!success) {
        ESP_LOGW(TAG, "ToF channel %d failed after %d retries", channel, max_retries);
    }

    data->distance_mm = measure.RangeMilliMeter;
    data->range_status = measure.RangeStatus;

    // Validate measurement:
    // 1. RangeStatus != 4 (not out of range)
    // 2. Distance < 65535 (not error value)
    // 3. Distance <= max threshold
    data->valid = (measure.RangeStatus != 4) &&
                  (measure.RangeMilliMeter < 65535) &&
                  (measure.RangeMilliMeter <= g_max_distance_mm);

    // If invalid, set distance to max threshold for safety
    if (!data->valid) {
        data->distance_mm = g_max_distance_mm;
    }

    xSemaphoreGive(tof_mutex);

#if USE_TCA9548A
    i2c_bus_unlock();
#endif

    return ESP_OK;
}

/**
 * @brief Read all ToF sensors at once
 */
esp_err_t tof_read_all(tof_data_t *top_data, tof_data_t *front_data,
                       tof_data_t *left_front_data, tof_data_t *left_rear_data)
{
	    // 目前硬件只有两个 ToF：SD1(Front)、SD2(Left-Front)
	    // 这里只实际读取这两个，其他通道保持原状态

	    if (tof_front_initialized && front_data) {
	        esp_err_t ret = tof_read(TOF_FRONT, front_data);
	        if (ret == ESP_OK) {
	            g_front_data = *front_data;
	        }
	    }

	    if (tof_left_front_initialized && left_front_data) {
	        esp_err_t ret = tof_read(TOF_LEFT_FRONT, left_front_data);
	        if (ret == ESP_OK) {
	            g_left_front_data = *left_front_data;
	        }
	    }

	    // top_data / left_rear_data 若非空，直接返回当前缓存（一般为无效）
	    if (top_data) {
	        *top_data = g_top_data;
	    }
	    if (left_rear_data) {
	        *left_rear_data = g_left_rear_data;
	    }

	    return ESP_OK;
}

/**
 * @brief Get top ToF distance
 */
uint16_t tof_get_top_distance(void)
{
    return g_top_data.valid ? g_top_data.distance_mm : 0xFFFF;
}

/**
 * @brief Get front ToF distance
 */
uint16_t tof_get_front_distance(void)
{
    return g_front_data.valid ? g_front_data.distance_mm : 0xFFFF;
}

/**
 * @brief Get left-front ToF distance
 */
uint16_t tof_get_left_front_distance(void)
{
    return g_left_front_data.valid ? g_left_front_data.distance_mm : 0xFFFF;
}

/**
 * @brief Get left-rear ToF distance
 */
uint16_t tof_get_left_rear_distance(void)
{
    return g_left_rear_data.valid ? g_left_rear_data.distance_mm : 0xFFFF;
}

/**
 * @brief Set maximum valid distance threshold
 */
void tof_set_max_distance(uint16_t max_distance_mm)
{
    g_max_distance_mm = max_distance_mm;
    ESP_LOGI(TAG, "ToF max distance set to %u mm", max_distance_mm);
}

/**
 * @brief Set channel switch delay
 */
void tof_set_channel_delay(uint8_t delay_ms)
{
    if (delay_ms > 10) {
        delay_ms = 10;  // Cap at 10ms
    }
    g_channel_delay_ms = delay_ms;
    ESP_LOGI(TAG, "ToF channel switch delay set to %u ms", delay_ms);
}

/**
 * @brief Backward-compatible helper: read distance by legacy index
 *
 * This function is provided for older example sketches (e.g.
 * examples/wall_following.ino) that used a simple index-based API:
 *   index 0 -> front sensor
 *   index 1 -> left-side sensor
 *   index 2 -> right-side sensor
 *
 * It maps those indices onto the new multi-sensor interface and applies the
 * same validation/filtering rules as tof_read(). New code should prefer the
 * typed APIs (tof_read() / tof_get_*_distance()).
 */
uint16_t tof_read_distance(uint8_t index)
{
    tof_position_t position;

    switch (index) {
        case 0:
            // Front ToF
            position = TOF_FRONT;
            break;
        case 1:
            // Left-side ToF
            position = TOF_LEFT_FRONT;
            break;
        case 2:
            // "Right" side ToF（当前映射到另一侧向传感器）
            position = TOF_LEFT_REAR;
            break;
        default:
            // Unsupported index
            return 0;
    }

    tof_data_t data;
    if (tof_read(position, &data) != ESP_OK || !data.valid) {
        // 与旧代码兼容：无效读数返回0
        return 0;
    }

    return data.distance_mm;
}

/* ========== 异步读取任务 / Async Reading Task ========== */

/**
 * @brief Async ToF reading task
 *
 * Continuously reads all ToF sensors in the background and updates cached data.
 * Other tasks can call tof_get_cached_*() to get latest readings without blocking.
 */
static void tof_async_reading_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ToF async reading task started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100); // 10Hz - 给足够时间读取

    while (tof_async_running) {
        // 读取前方ToF (SD1)
        if (tof_front_initialized) {
            tof_data_t data;
            if (tof_read(TOF_FRONT, &data) == ESP_OK) {
                data.timestamp_ms = millis();

                if (xSemaphoreTake(tof_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_front_data = data;
                    xSemaphoreGive(tof_mutex);
                }
            }
        }

        // 读取左前ToF (SD2)
        if (tof_left_front_initialized) {
            tof_data_t data;
            if (tof_read(TOF_LEFT_FRONT, &data) == ESP_OK) {
                data.timestamp_ms = millis();

                if (xSemaphoreTake(tof_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_left_front_data = data;
                    xSemaphoreGive(tof_mutex);
                }
            }
        }

        // 等待下一个周期
        vTaskDelayUntil(&last_wake_time, frequency);
    }

    ESP_LOGI(TAG, "ToF async reading task stopped");
    tof_async_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Start asynchronous ToF reading task
 */
esp_err_t tof_start_async_reading(void)
{
    if (tof_async_running) {
        ESP_LOGW(TAG, "Async reading task already running");
        return ESP_OK;
    }

    tof_async_running = true;

    BaseType_t ret = xTaskCreatePinnedToCore(
        tof_async_reading_task,
        "tof_async",
        4096,
        NULL,
        3,  // Priority 3 (same as sensor tasks)
        &tof_async_task_handle,
        0   // Core 0 (same as other sensor tasks)
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create async reading task");
        tof_async_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ToF async reading task created");
    return ESP_OK;
}

/**
 * @brief Stop asynchronous ToF reading task
 */
void tof_stop_async_reading(void)
{
    if (!tof_async_running) {
        return;
    }

    ESP_LOGI(TAG, "Stopping ToF async reading task...");
    tof_async_running = false;

    // 等待任务结束（最多1秒）
    uint32_t timeout = 1000;
    while (tof_async_task_handle != NULL && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout -= 10;
    }

    if (tof_async_task_handle != NULL) {
        ESP_LOGW(TAG, "Force deleting async reading task");
        vTaskDelete(tof_async_task_handle);
        tof_async_task_handle = NULL;
    }

    ESP_LOGI(TAG, "ToF async reading task stopped");
}

/**
 * @brief Get cached ToF data (non-blocking)
 */
esp_err_t tof_get_cached(tof_position_t position, tof_data_t *data)
{
    if (data == NULL) {
        return ESP_FAIL;
    }

    if (xSemaphoreTake(tof_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_FAIL;
    }

    switch (position) {
        case TOF_TOP:
            *data = g_top_data;
            break;
        case TOF_FRONT:
            *data = g_front_data;
            break;
        case TOF_LEFT_FRONT:
            *data = g_left_front_data;
            break;
        case TOF_LEFT_REAR:
            *data = g_left_rear_data;
            break;
        default:
            xSemaphoreGive(tof_mutex);
            return ESP_FAIL;
    }

    xSemaphoreGive(tof_mutex);
    return ESP_OK;
}

/**
 * @brief Get cached front ToF distance (non-blocking)
 */
uint16_t tof_get_cached_front_distance(void)
{
    tof_data_t data;
    if (tof_get_cached(TOF_FRONT, &data) == ESP_OK && data.valid) {
        return data.distance_mm;
    }
    return 0xFFFF;
}

/**
 * @brief Get cached left-front ToF distance (non-blocking)
 */
uint16_t tof_get_cached_left_front_distance(void)
{
    tof_data_t data;
    if (tof_get_cached(TOF_LEFT_FRONT, &data) == ESP_OK && data.valid) {
        return data.distance_mm;
    }
    return 0xFFFF;
}

/**
 * @brief Check if cached data is fresh
 */
bool tof_is_data_fresh(tof_position_t position, uint32_t timeout_ms)
{
    tof_data_t data;
    if (tof_get_cached(position, &data) != ESP_OK) {
        return false;
    }

    if (!data.valid) {
        return false;
    }

    uint32_t age_ms = millis() - data.timestamp_ms;
    return age_ms <= timeout_ms;
}


