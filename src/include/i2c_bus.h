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

#ifdef __cplusplus
}
#endif

#endif // I2C_BUS_H

