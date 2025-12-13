/**
 * @file servo_control.cpp
 * @brief Servo Control Implementation using ESP-IDF LEDC
 *
 * Modified to use direct ESP-IDF driver calls to avoid timer conflicts
 * with motor_driver.cpp which uses Timer 0 and Timer 1.
 */

#include "include/servo_control.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <Arduino.h>

#define ALLOCATED_SERVO_PIN 11

// Use LEDC Timer 2 and Channel 2 to avoid conflict with Motor Driver (Timer 0 &
// 1)
#define SERVO_LEDC_TIMER LEDC_TIMER_2
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_2
#define SERVO_FREQ 50
// Increase resolution to 14 bits to allow APB clock usage for 50Hz.
// With 10 bits: Div = 80M/(50*1024) = 1562500 (Overflows hw limit sometimes,
// forces Slow Clock) With 14 bits: Div = 80M/(50*16384) = 97.6 (Fits APB Clock)
#define SERVO_RES LEDC_TIMER_14_BIT

static const char *TAG = "SERVO";
static bool g_servo_inited = false;

esp_err_t servo_init(void) {
  if (g_servo_inited)
    return ESP_OK;

  ESP_LOGI(TAG, "Initializing Servo on Pin %d...", ALLOCATED_SERVO_PIN);

  // Configure LEDC Timer
  // We use Timer 2 to avoid conflict with Timer 0 and 1 used by motor_driver
  // LEDC_AUTO_CLK will now pick APB (Source 4) because 14-bit resolution makes
  // the divider fit.
  ledc_timer_config_t ledc_timer = {.speed_mode = SERVO_LEDC_MODE,
                                    .duty_resolution = SERVO_RES,
                                    .timer_num = SERVO_LEDC_TIMER,
                                    .freq_hz = SERVO_FREQ,
                                    .clk_cfg = LEDC_AUTO_CLK};

  esp_err_t ret = ledc_timer_config(&ledc_timer);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC timer! Error: %s",
             esp_err_to_name(ret));
    return ret;
  }

  // Configure LEDC Channel
  ledc_channel_config_t ledc_channel = {.gpio_num = ALLOCATED_SERVO_PIN,
                                        .speed_mode = SERVO_LEDC_MODE,
                                        .channel = SERVO_LEDC_CHANNEL,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .timer_sel = SERVO_LEDC_TIMER,
                                        .duty = 0,
                                        .hpoint = 0};

  ret = ledc_channel_config(&ledc_channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC channel! Error: %s",
             esp_err_to_name(ret));
    return ret;
  }

  // Initialize to 90 degrees (middle)
  servo_set_angle(90);

  g_servo_inited = true;
  ESP_LOGI(TAG,
           "Servo initialized successfully on Timer %d, Channel %d (14-bit)",
           SERVO_LEDC_TIMER, SERVO_LEDC_CHANNEL);
  return ESP_OK;
}

esp_err_t servo_set_angle(int angle) {
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;

  // Calculate duty cycle for MG996R (0.5ms - 2.5ms)
  // Period = 20ms (50Hz)
  // Resolution = 14 bits (16384 steps)

  // Pulse width in microseconds
  float pulse_width_us = (float)angle / 180.0f * 2000.0f + 500.0f;

  // Duty cycle value
  // (pulse_width_us / 20000.0f) * 16384.0f
  uint32_t duty = (uint32_t)((pulse_width_us / 20000.0f) * 16384.0f);

  if (duty >= (1 << 14))
    duty = (1 << 14) - 1; // Clamp to max resolution value

  // Set duty
  ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
  ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);

  // ESP_LOGD(TAG, "Servo set to %d deg (duty: %lu)", angle, duty);
  return ESP_OK;
}

static bool g_sweep_active = false;

void servo_stop_sweep(void) { g_sweep_active = false; }

void servo_sweep_0_60(void) {
  ESP_LOGI(TAG, "Starting Servo Sweep 0 -> 60 -> 0 (Continuous)");
  g_sweep_active = true;

  while (g_sweep_active) {
    // 0 to 60
    for (int i = 0; i <= 60; i += 2) {
      if (!g_sweep_active)
        break;
      servo_set_angle(i);
      vTaskDelay(pdMS_TO_TICKS(20)); // Slow smooth movement
    }

    if (!g_sweep_active)
      break;
    vTaskDelay(pdMS_TO_TICKS(500)); // Hold at 60

    // 60 to 0
    for (int i = 60; i >= 0; i -= 2) {
      if (!g_sweep_active)
        break;
      servo_set_angle(i);
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (!g_sweep_active)
      break;
    servo_set_angle(0);             // Ensure 0
    vTaskDelay(pdMS_TO_TICKS(500)); // Hold at 0 before repeating
  }

  // Ensure stopped safely
  ESP_LOGI(TAG, "Servo Sweep Stopped");
}
