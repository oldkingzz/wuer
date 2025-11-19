/**
 * @file vive_sensor.cpp
 * @brief Vive Positioning System Implementation
 */

#include "Arduino.h"
#include "include/vive_sensor.h"
#include "include/vive510.h"
#include "include/gpio_config.h"
#include "esp_log.h"

static const char *TAG = "VIVE_SENSOR";

// Vive sensor objects
static Vive510 *vive1 = NULL;
static Vive510 *vive2 = NULL;

// Initialization status
static bool vive1_initialized = false;
static bool vive2_initialized = false;

// Latest measurements with median filtering
static vive_data_t g_vive1_data = {0, 0, false, 0};
static vive_data_t g_vive2_data = {0, 0, false, 0};

// Median filter buffers for sensor 1
static uint16_t vive1_x_buf[3] = {0, 0, 0};
static uint16_t vive1_y_buf[3] = {0, 0, 0};

// Median filter buffers for sensor 2
static uint16_t vive2_x_buf[3] = {0, 0, 0};
static uint16_t vive2_y_buf[3] = {0, 0, 0};

// Mutex for thread safety
static SemaphoreHandle_t vive_mutex = NULL;

/**
 * @brief 3-point median filter
 */
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t middle;
    if ((a <= b) && (a <= c))
        middle = (b <= c) ? b : c;
    else if ((b <= a) && (b <= c))
        middle = (a <= c) ? a : c;
    else
        middle = (a <= b) ? a : b;
    return middle;
}

/**
 * @brief Initialize both Vive sensors
 */
esp_err_t vive_init(void)
{
    ESP_LOGI(TAG, "Initializing Vive sensors...");
    Serial.println("Initializing Vive positioning sensors...");
    Serial.flush();

    // Create mutex
    vive_mutex = xSemaphoreCreateMutex();
    if (vive_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        Serial.println("ERROR: Failed to create Vive mutex");
        return ESP_FAIL;
    }

    // Initialize Vive sensor 1
    Serial.print("Init Vive Sensor 1 (GPIO ");
    Serial.print(VIVE1_SIGNAL_GPIO);
    Serial.print(")... ");
    Serial.flush();

    vive1 = new Vive510(VIVE1_SIGNAL_GPIO);
    if (vive1 == NULL) {
        ESP_LOGE(TAG, "Failed to create Vive1 object");
        Serial.println("Failed!");
        vive1_initialized = false;
    } else {
        vive1->begin();
        ESP_LOGI(TAG, "Vive sensor 1 initialized");
        Serial.println("Success!");
        vive1_initialized = true;
    }

    // Initialize Vive sensor 2
    Serial.print("Init Vive Sensor 2 (GPIO ");
    Serial.print(VIVE2_SIGNAL_GPIO);
    Serial.print(")... ");
    Serial.flush();

    vive2 = new Vive510(VIVE2_SIGNAL_GPIO);
    if (vive2 == NULL) {
        ESP_LOGE(TAG, "Failed to create Vive2 object");
        Serial.println("Failed!");
        vive2_initialized = false;
    } else {
        vive2->begin();
        ESP_LOGI(TAG, "Vive sensor 2 initialized");
        Serial.println("Success!");
        vive2_initialized = true;
    }

    if (vive1_initialized || vive2_initialized) {
        ESP_LOGI(TAG, "Vive sensors initialization complete");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "All Vive sensors failed to initialize");
        return ESP_FAIL;
    }
}

/**
 * @brief Read data from a specific Vive sensor
 */
esp_err_t vive_read(vive_sensor_id_t sensor_id, vive_data_t *data)
{
    if (data == NULL) return ESP_FAIL;
    if (vive_mutex == NULL) return ESP_FAIL;

    xSemaphoreTake(vive_mutex, portMAX_DELAY);

    Vive510 *sensor = NULL;
    bool initialized = false;
    uint16_t *x_buf = NULL;
    uint16_t *y_buf = NULL;
    vive_data_t *global_data = NULL;

    // Select sensor
    if (sensor_id == VIVE_SENSOR_1) {
        sensor = vive1;
        initialized = vive1_initialized;
        x_buf = vive1_x_buf;
        y_buf = vive1_y_buf;
        global_data = &g_vive1_data;
    } else if (sensor_id == VIVE_SENSOR_2) {
        sensor = vive2;
        initialized = vive2_initialized;
        x_buf = vive2_x_buf;
        y_buf = vive2_y_buf;
        global_data = &g_vive2_data;
    } else {
        xSemaphoreGive(vive_mutex);
        return ESP_FAIL;
    }

    // Check if sensor is initialized
    if (!initialized || sensor == NULL) {
        data->x = 0;
        data->y = 0;
        data->valid = false;
        data->status = 0;
        xSemaphoreGive(vive_mutex);
        return ESP_FAIL;
    }

    // Read sensor status and coordinates
    int status = sensor->status();
    data->status = status;

    if (status == VIVE_RECEIVING) {
        // Shift buffer
        x_buf[2] = x_buf[1];
        x_buf[1] = x_buf[0];
        y_buf[2] = y_buf[1];
        y_buf[1] = y_buf[0];

        // Read new values
        x_buf[0] = sensor->xCoord();
        y_buf[0] = sensor->yCoord();

        // Apply median filter
        uint16_t x_filtered = median3(x_buf[0], x_buf[1], x_buf[2]);
        uint16_t y_filtered = median3(y_buf[0], y_buf[1], y_buf[2]);

        // Validate range (1000-8000 is valid range)
        if (x_filtered > 8000 || y_filtered > 8000 || x_filtered < 1000 || y_filtered < 1000) {
            data->x = 0;
            data->y = 0;
            data->valid = false;
        } else {
            data->x = x_filtered;
            data->y = y_filtered;
            data->valid = true;
        }
    } else {
        // Not receiving valid signal
        data->x = 0;
        data->y = 0;
        data->valid = false;

        // Try to sync
        sensor->sync(5);
    }

    // Update global data
    *global_data = *data;

    xSemaphoreGive(vive_mutex);
    return ESP_OK;
}

/**
 * @brief Read data from both Vive sensors
 */
esp_err_t vive_read_all(vive_data_t *sensor1_data, vive_data_t *sensor2_data)
{
    esp_err_t ret1 = vive_read(VIVE_SENSOR_1, sensor1_data);
    esp_err_t ret2 = vive_read(VIVE_SENSOR_2, sensor2_data);

    if (ret1 == ESP_OK || ret2 == ESP_OK) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

/**
 * @brief Get X coordinate from sensor 1
 */
uint16_t vive_get_sensor1_x(void)
{
    return g_vive1_data.valid ? g_vive1_data.x : 0;
}

/**
 * @brief Get Y coordinate from sensor 1
 */
uint16_t vive_get_sensor1_y(void)
{
    return g_vive1_data.valid ? g_vive1_data.y : 0;
}

/**
 * @brief Get X coordinate from sensor 2
 */
uint16_t vive_get_sensor2_x(void)
{
    return g_vive2_data.valid ? g_vive2_data.x : 0;
}

/**
 * @brief Get Y coordinate from sensor 2
 */
uint16_t vive_get_sensor2_y(void)
{
    return g_vive2_data.valid ? g_vive2_data.y : 0;
}

/**
 * @brief Check if sensor 1 is receiving valid signal
 */
bool vive_sensor1_is_valid(void)
{
    return g_vive1_data.valid;
}

/**
 * @brief Check if sensor 2 is receiving valid signal
 */
bool vive_sensor2_is_valid(void)
{
    return g_vive2_data.valid;
}


