/**
 * @file tophat.h
 * @brief Driver for the MEAM5100 Tophat module
 */

#ifndef TOPHAT_H
#define TOPHAT_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize the Tophat driver
 * @return ESP_OK or ESP_FAIL
 */
esp_err_t tophat_init(void);

/**
 * @brief Send joystick/command packet count to Tophat
 *
 * Must be called at least twice per second (2Hz) to prevent
 * Tophat from timing out.
 *
 * @param packet_count Number of valid command packets received since last call
 * @return ESP_OK or ESP_FAIL
 */
esp_err_t tophat_send_heartbeat(uint32_t packet_count);

/**
 * @brief Check if robot is currently penalized (out of health)
 * @return true if penalized, false otherwise
 */
bool tophat_is_penalized(void);

#endif // TOPHAT_H
