#include "include/i2c_bus.h"

static SemaphoreHandle_t g_i2c_bus_mutex = NULL;

void i2c_bus_init(void) {
  if (g_i2c_bus_mutex == NULL) {
    g_i2c_bus_mutex = xSemaphoreCreateMutex();
  }
}

void i2c_bus_lock(void) {
  if (g_i2c_bus_mutex) {
    xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY);
  }
}

void i2c_bus_unlock(void) {
  if (g_i2c_bus_mutex) {
    xSemaphoreGive(g_i2c_bus_mutex);
  }
}

// Add these implementations
#include "esp_err.h"
#include <Arduino.h>
#include <Wire.h>

int i2c_bus_write_byte(uint8_t addr, uint8_t data) {
  i2c_bus_lock();
  Wire.beginTransmission(addr);
  Wire.write(data);
  uint8_t ret = Wire.endTransmission();
  i2c_bus_unlock();
  return (ret == 0) ? ESP_OK : ESP_FAIL;
}

int i2c_bus_read_byte(uint8_t addr, uint8_t *data) {
  i2c_bus_lock();
  // Usually protocol is: Read from address.
  // Standard Wire.requestFrom handles START -> ADDR+R -> DATA -> STOP
  uint8_t len = Wire.requestFrom(addr, (uint8_t)1);
  if (len > 0 && Wire.available()) {
    *data = Wire.read();
    i2c_bus_unlock();
    return ESP_OK;
  }
  i2c_bus_unlock();
  return ESP_FAIL;
}
