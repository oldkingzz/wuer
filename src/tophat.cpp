/**
 * @file tophat.cpp
 * @brief Driver for the MEAM5100 Tophat module
 */

#include "include/tophat.h"
#include "esp_log.h"
#include "include/i2c_bus.h"
#include "include/web.h" // To access global packet counters if implemented there
#include <Arduino.h>

static const char *TAG = "TOPHAT";
static const uint8_t TOPHAT_ADDR = 0x28; // I2C Address

// Global state
static bool g_tophat_inited = false;
// Helper to select TCA channel
#include <Wire.h>

static esp_err_t tca_select(uint8_t channel) {
  if (channel > 7)
    return ESP_FAIL;
  i2c_bus_lock();
  Wire.beginTransmission(0x70); // TCA9548A address
  Wire.write(1 << channel);
  uint8_t ret = Wire.endTransmission();
  i2c_bus_unlock();
  if (ret != 0) {
    // Serial.printf("[TOPHAT] TCA Select Failed! Ret=%d\n", ret);
    return ESP_FAIL;
  }
  return ESP_OK;
}

static bool g_is_penalized = false;
static uint32_t g_penalty_start_time = 0;
static const uint32_t PENALTY_DURATION_MS = 15000; // 15 seconds

// Scan for Tophat on all TCA channels
static int8_t g_tophat_channel = -1;

esp_err_t tophat_init(void) {
  if (g_tophat_inited)
    return ESP_OK;

  Serial.println("[TOPHAT] Initializing Tophat (0x28)...");

  // 1. Try Direct Communication first
  i2c_bus_lock();
  Wire.beginTransmission(TOPHAT_ADDR);
  uint8_t ret = Wire.endTransmission();

  // If first ping failed, try writing a byte (mimicking successful user code)
  if (ret != 0) {
    // Serial.printf("[TOPHAT] Direct Ping failed (ret=%d), trying with
    // write...\n", ret);
    delay(10);
    Wire.beginTransmission(TOPHAT_ADDR);
    Wire.write(0x00); // Dummy byte
    ret = Wire.endTransmission();
  }
  i2c_bus_unlock();

  if (ret == 0) {
    Serial.println("[TOPHAT] Found Tophat on Main Bus!");
    g_tophat_inited = true;
    g_tophat_channel = -1; // -1 indicates Direct/Main Bus
    return ESP_OK;
  } else {
    Serial.printf("[TOPHAT] Direct Scan Failed. Error Code: %d\n", ret);
  }

  // 2. If direct failed, try scanning TCA channels
  Serial.println("[TOPHAT] Direct access failed, scanning TCA channels...");
  for (uint8_t ch = 0; ch < 8; ch++) {
    i2c_bus_lock();

    // Select Channel
    Wire.beginTransmission(0x70);
    Wire.write(1 << ch);
    if (Wire.endTransmission() != 0) {
      i2c_bus_unlock();
      continue;
    }

    // Ping Tophat
    Wire.beginTransmission(TOPHAT_ADDR);
    ret = Wire.endTransmission();

    i2c_bus_unlock();

    if (ret == 0) {
      Serial.printf("[TOPHAT] Found Tophat on TCA Channel %d!\n", ch);
      g_tophat_channel = ch;
      g_tophat_inited = true;
      return ESP_OK;
    }
  }

  Serial.println("[TOPHAT] ERROR: Tophat NOT found!");
  return ESP_FAIL;
}

esp_err_t tophat_send_heartbeat(uint32_t packet_count) {
  if (!g_tophat_inited) {
    Serial.println("[TOPHAT] Not inited!");
    return ESP_FAIL;
  }

  uint8_t count_byte = (uint8_t)(packet_count > 255 ? 255 : packet_count);

  // ATOMIC OPERATION START
  i2c_bus_lock();

  // 1. Select the correct channel if using TCA
  if (g_tophat_channel >= 0) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << g_tophat_channel);
    if (Wire.endTransmission() != 0) {
      i2c_bus_unlock();
      Serial.println("[TOPHAT] TCA Select Failed in heartbeat func");
      return ESP_FAIL;
    }
  }

  // 2. Write Heartbeat Packet
  Wire.beginTransmission(TOPHAT_ADDR);
  Wire.write(count_byte);
  uint8_t write_ret = Wire.endTransmission();

  if (write_ret != 0) {
    i2c_bus_unlock();
    Serial.printf("[TOPHAT] Failed to write heartbeat! Ret=%d\n", write_ret);
    return ESP_FAIL;
  }

  // 3. Read Status (Health)
  uint8_t len = Wire.requestFrom(TOPHAT_ADDR, (uint8_t)1);
  int status = -1;
  if (len > 0 && Wire.available()) {
    status = Wire.read();
  }

  // ATOMIC OPERATION END
  i2c_bus_unlock();

  if (status != -1) {
    // Debug
    // Serial.printf("[TOPHAT] Health: %d\n", status);

    static uint8_t last_status = 255;
    if (status == 0) {
      if (!g_is_penalized) {
        Serial.println("[TOPHAT] Health dropped to 0! PENALTY START.");
        g_is_penalized = true;
        g_penalty_start_time = millis();
      }
    }
    last_status = status;
    return ESP_OK;
  } else {
    Serial.println("[TOPHAT] Failed to read health!");
    return ESP_FAIL;
  }
}

bool tophat_is_penalized(void) { return g_is_penalized; }
