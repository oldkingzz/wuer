#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize global I2C/TCA bus mutex.
 *
 * Call this once after Wire.begin().
 */
void i2c_bus_init(void);

/**
 * @brief Lock the shared I2C/TCA bus.
 */
void i2c_bus_lock(void);

/**
 * @brief Unlock the shared I2C/TCA bus.
 */
void i2c_bus_unlock(void);

/**
 * @brief Write a single byte to an I2C device
 * @param addr I2C address
 * @param data Byte to write
 * @return ESP_OK or ESP_FAIL
 */
int i2c_bus_write_byte(uint8_t addr, uint8_t data);

/**
 * @brief Read a single byte from an I2C device
 * @param addr I2C address
 * @param data Pointer to store read byte
 * @return ESP_OK or ESP_FAIL
 */
int i2c_bus_read_byte(uint8_t addr, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // I2C_BUS_H
