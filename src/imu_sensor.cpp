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
static bool use_manual_mode = false;  // True if Adafruit library fails

// MPU6050 configuration
static uint8_t mpu_channel = IMU_CHANNEL;  // Will be auto-detected if needed
static uint8_t mpu_address = 0x68;         // Default MPU6050 address

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
    delay(10);  // Ensure channel switch completes
}

/**
 * @brief Read 16-bit signed value from MPU6050 register
 */
static int16_t read_register16(uint8_t reg)
{
#if USE_TCA9548A
    tca_select(mpu_channel);
    delay(5);
#endif

    Wire.beginTransmission(mpu_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu_address, (uint8_t)2);

    if (Wire.available() >= 2) {
        int16_t value = Wire.read() << 8 | Wire.read();
        return value;
    }
    return 0;
}

/**
 * @brief Auto-detect MPU6050 on TCA9548A channels
 * @return true if found, false otherwise
 */
static bool auto_detect_mpu6050(void)
{
#if USE_TCA9548A
    ESP_LOGI(TAG, "Auto-detecting MPU6050 on TCA9548A channels...");

    for (uint8_t ch = 0; ch < 8; ch++) {
        tca_select(ch);
        delay(50);

        for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
            Wire.beginTransmission(addr);
            uint8_t error = Wire.endTransmission();

            if (error == 0) {
                mpu_channel = ch;
                mpu_address = addr;
                ESP_LOGI(TAG, "Found MPU6050 at channel %d, address 0x%02X", ch, addr);
                Serial.print("✓ Found MPU6050 on channel ");
                Serial.print(ch);
                Serial.print(" at 0x");
                Serial.println(addr, HEX);
                return true;
            }
        }
    }

    ESP_LOGE(TAG, "MPU6050 not found on any channel!");
    return false;
#else
    // Direct connection mode - just check 0x68 and 0x69
    for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            mpu_address = addr;
            ESP_LOGI(TAG, "Found MPU6050 at address 0x%02X", addr);
            return true;
        }
    }

    ESP_LOGE(TAG, "MPU6050 not found!");
    return false;
#endif
}

/**
 * @brief Initialize IMU sensor
 */
esp_err_t imu_init(void)
{
    ESP_LOGI(TAG, "Initializing IMU sensor...");

    // Create mutex
    imu_mutex = xSemaphoreCreateMutex();
    if (imu_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Auto-detect MPU6050
    Serial.println("Detecting MPU6050...");
    Serial.flush();

    if (!auto_detect_mpu6050()) {
        ESP_LOGE(TAG, "MPU6050 not found!");
        Serial.println("✗ MPU6050 not found!");
        Serial.flush();
        imu_initialized = false;
        return ESP_FAIL;
    }

    Serial.print("Init MPU6050 (Channel ");
    Serial.print(mpu_channel);
    Serial.print(", 0x");
    Serial.print(mpu_address, HEX);
    Serial.println(")...");
    Serial.flush();

    // Wake up MPU6050
    Serial.print("  Waking up MPU6050... ");
    Serial.flush();

#if USE_TCA9548A
    tca_select(mpu_channel);
    delay(50);
#endif

    Wire.beginTransmission(mpu_address);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Clear SLEEP bit
    uint8_t error = Wire.endTransmission();

    if (error != 0) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050, error: %d", error);
        Serial.print("Failed! Error: ");
        Serial.println(error);
        Serial.flush();
        imu_initialized = false;
        return ESP_FAIL;
    }

    Serial.println("OK!");
    Serial.flush();
    delay(100);

    // Try to initialize Adafruit library
    Serial.print("  Initializing Adafruit library... ");
    Serial.flush();

#if USE_TCA9548A
    tca_select(mpu_channel);
    delay(100);
#endif

    if (!mpu.begin(mpu_address, &Wire)) {
        ESP_LOGW(TAG, "Adafruit library init failed, using manual mode");
        Serial.println("Failed!");
        Serial.println("  → Switching to manual register mode");
        Serial.flush();
        use_manual_mode = true;
    } else {
        ESP_LOGI(TAG, "Adafruit library initialized");
        Serial.println("Success!");
        Serial.flush();
        use_manual_mode = false;
    }

    // Configure MPU6050
    Serial.print("  Configuring MPU6050... ");
    Serial.flush();

#if USE_TCA9548A
    tca_select(mpu_channel);
    delay(50);
#endif

    if (use_manual_mode) {
        // Manual configuration via registers
        // Set accelerometer range to ±8g (register 0x1C, value 0x10)
        Wire.beginTransmission(mpu_address);
        Wire.write(0x1C);  // ACCEL_CONFIG
        Wire.write(0x10);  // ±8g
        Wire.endTransmission();
        delay(10);

#if USE_TCA9548A
        tca_select(mpu_channel);
        delay(10);
#endif

        // Set gyro range to ±500°/s (register 0x1B, value 0x08)
        Wire.beginTransmission(mpu_address);
        Wire.write(0x1B);  // GYRO_CONFIG
        Wire.write(0x08);  // ±500°/s
        Wire.endTransmission();
        delay(10);

        ESP_LOGI(TAG, "Manual config: Accel=±8g, Gyro=±500°/s");
    } else {
        // Adafruit library configuration
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        delay(10);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        delay(10);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        delay(10);

        ESP_LOGI(TAG, "Adafruit config: Accel=8G, Gyro=500°/s, Filter=21Hz");
    }

    Serial.println("Done!");
    Serial.flush();
    delay(100);

    // Test read
    Serial.print("  Testing read... ");
    Serial.flush();

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;

    if (use_manual_mode) {
        // Manual read
#if USE_TCA9548A
        tca_select(mpu_channel);
        delay(50);
#endif

        // Read temperature
        int16_t temp_raw = read_register16(0x41);
        temperature = (temp_raw / 340.0) + 36.53;

        // Read accelerometer (±8g: 4096 LSB/g)
        int16_t ax = read_register16(0x3B);
        int16_t ay = read_register16(0x3D);
        int16_t az = read_register16(0x3F);
        accel_x = (ax / 4096.0) * 9.81;
        accel_y = (ay / 4096.0) * 9.81;
        accel_z = (az / 4096.0) * 9.81;

        // Read gyroscope (±500°/s: 65.5 LSB/(°/s))
        int16_t gx = read_register16(0x43);
        int16_t gy = read_register16(0x45);
        int16_t gz = read_register16(0x47);
        gyro_x = (gx / 65.5) * (3.14159 / 180.0);
        gyro_y = (gy / 65.5) * (3.14159 / 180.0);
        gyro_z = (gz / 65.5) * (3.14159 / 180.0);

    } else {
        // Adafruit library read
#if USE_TCA9548A
        tca_select(mpu_channel);
        delay(50);
#endif

        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);

        accel_x = a.acceleration.x;
        accel_y = a.acceleration.y;
        accel_z = a.acceleration.z;
        gyro_x = g.gyro.x;
        gyro_y = g.gyro.y;
        gyro_z = g.gyro.z;
        temperature = t.temperature;
    }

    // Display test data
    Serial.print("Accel(");
    Serial.print(accel_x, 2);
    Serial.print(", ");
    Serial.print(accel_y, 2);
    Serial.print(", ");
    Serial.print(accel_z, 2);
    Serial.print(") Gyro(");
    Serial.print(gyro_x, 3);
    Serial.print(", ");
    Serial.print(gyro_y, 3);
    Serial.print(", ");
    Serial.print(gyro_z, 3);
    Serial.print(") Temp=");
    Serial.print(temperature, 1);
    Serial.println("°C");

    if (accel_z > 5.0 && accel_z < 15.0) {
        Serial.println("  ✓ Data looks good!");
    } else {
        Serial.println("  ⚠ WARNING: Z acceleration unexpected (expected ~9.8 m/s²)");
    }
    Serial.flush();

    imu_initialized = true;
    ESP_LOGI(TAG, "IMU initialization complete (mode: %s)", use_manual_mode ? "manual" : "Adafruit");
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

    if (use_manual_mode) {
        // Manual register reading mode
#if USE_TCA9548A
        tca_select(mpu_channel);
        delay(10);
#endif

        // Read temperature (register 0x41-0x42)
        int16_t temp_raw = read_register16(0x41);
        data->temperature = (temp_raw / 340.0) + 36.53;

        // Read accelerometer (registers 0x3B-0x40, ±8g: 4096 LSB/g)
        int16_t ax = read_register16(0x3B);
        int16_t ay = read_register16(0x3D);
        int16_t az = read_register16(0x3F);
        data->accel.x = (ax / 4096.0) * 9.81;
        data->accel.y = (ay / 4096.0) * 9.81;
        data->accel.z = (az / 4096.0) * 9.81;

        // Read gyroscope (registers 0x43-0x48, ±500°/s: 65.5 LSB/(°/s))
        int16_t gx = read_register16(0x43);
        int16_t gy = read_register16(0x45);
        int16_t gz = read_register16(0x47);
        data->gyro.x = (gx / 65.5) * (3.14159 / 180.0);
        data->gyro.y = (gy / 65.5) * (3.14159 / 180.0);
        data->gyro.z = (gz / 65.5) * (3.14159 / 180.0);

        data->valid = true;

    } else {
        // Adafruit library mode
#if USE_TCA9548A
        tca_select(mpu_channel);
        delay(10);
#endif

        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        data->accel.x = accel.acceleration.x;
        data->accel.y = accel.acceleration.y;
        data->accel.z = accel.acceleration.z;

        data->gyro.x = gyro.gyro.x;
        data->gyro.y = gyro.gyro.y;
        data->gyro.z = gyro.gyro.z;

        data->temperature = temp.temperature;
        data->valid = true;
    }

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

