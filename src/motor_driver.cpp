/**
 * @file motor_driver.cpp
 * @brief 电机驱动器实现 / Motor Driver Implementation
 *
 * 实现L298N电机驱动器的控制功能
 * Implements control functions for L298N motor driver
 */

#include <string.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/motor_driver.h"
#include "include/gpio_config.h"

static const char *TAG = "MOTOR_DRIVER";

// 电机状态全局变量 / Global motor state variable
static motor_state_t g_motor_state = {
    .is_running = false,
    .direction = MOTOR_STOP,
    .duty_cycle = 0,
    .speed_percentage = 0.0f
};

// LEDC PWM配置 / LEDC PWM Configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           MOTOR_PWM_RESOLUTION
#define LEDC_FREQUENCY          MOTOR_PWM_FREQ_HZ

/**
 * @brief 初始化电机驱动器
 */
esp_err_t motor_driver_init(void)
{
    ESP_LOGI(TAG, "初始化电机驱动器... / Initializing motor driver...");
    
    // 配置方向控制引脚 (IN1, IN2) / Configure direction control pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1_GPIO) | (1ULL << MOTOR_IN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO配置失败 / GPIO configuration failed");
        return ret;
    }
    
    // 初始化方向引脚为停止状态 / Initialize direction pins to stop state
    gpio_set_level(MOTOR_IN1_GPIO, 0);
    gpio_set_level(MOTOR_IN2_GPIO, 0);
    
    // 配置LEDC定时器 / Configure LEDC timer
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_MODE;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.freq_hz = LEDC_FREQUENCY;
    ledc_timer.timer_num = LEDC_TIMER;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;

    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC定时器配置失败 / LEDC timer configuration failed");
        return ret;
    }

    // 配置LEDC通道 / Configure LEDC channel
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num = MOTOR_PWM_GPIO;
    ledc_channel.speed_mode = LEDC_MODE;
    ledc_channel.channel = LEDC_CHANNEL;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel = LEDC_TIMER;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;

    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC通道配置失败 / LEDC channel configuration failed");
        return ret;
    }
    
    ESP_LOGI(TAG, "电机驱动器初始化成功 / Motor driver initialized successfully");
    ESP_LOGI(TAG, "PWM频率: %d Hz, 分辨率: %d 位", LEDC_FREQUENCY, (1 << LEDC_DUTY_RES) - 1);
    
    return ESP_OK;
}

/**
 * @brief 设置电机方向
 */
esp_err_t motor_set_direction(motor_direction_t direction)
{
    // 安全检查：如果要改变方向（不是停止），必须先确保电机已停止
    // Safety check: If changing direction (not stopping), motor must be stopped first
    if (direction != MOTOR_STOP && direction != g_motor_state.direction) {
        if (g_motor_state.is_running && g_motor_state.duty_cycle > 0) {
            ESP_LOGE(TAG, "安全错误：电机必须先停止才能改变方向！");
            ESP_LOGE(TAG, "Safety Error: Motor must be stopped before changing direction!");
            return ESP_FAIL;
        }
    }

    switch (direction) {
        case MOTOR_STOP:
            // 停止: IN1=0, IN2=0 / Stop: IN1=0, IN2=0
            gpio_set_level(MOTOR_IN1_GPIO, 0);
            gpio_set_level(MOTOR_IN2_GPIO, 0);
            ESP_LOGI(TAG, "电机方向: 停止 / Motor direction: STOP");
            break;

        case MOTOR_FORWARD:
            // 正转: IN1=1, IN2=0 / Forward: IN1=1, IN2=0
            gpio_set_level(MOTOR_IN1_GPIO, 1);
            gpio_set_level(MOTOR_IN2_GPIO, 0);
            ESP_LOGI(TAG, "电机方向: 正转 / Motor direction: FORWARD");
            break;

        case MOTOR_BACKWARD:
            // 反转: IN1=0, IN2=1 / Backward: IN1=0, IN2=1
            gpio_set_level(MOTOR_IN1_GPIO, 0);
            gpio_set_level(MOTOR_IN2_GPIO, 1);
            ESP_LOGI(TAG, "电机方向: 反转 / Motor direction: BACKWARD");
            break;

        default:
            ESP_LOGE(TAG, "无效的电机方向 / Invalid motor direction");
            return ESP_ERR_INVALID_ARG;
    }

    g_motor_state.direction = direction;
    return ESP_OK;
}

/**
 * @brief 设置电机速度 (PWM占空比)
 */
esp_err_t motor_set_speed(uint32_t duty_cycle)
{
    // 检查占空比范围 / Check duty cycle range
    if (duty_cycle > MOTOR_PWM_MAX_DUTY) {
        ESP_LOGE(TAG, "占空比超出范围: %lu (最大: %d)", duty_cycle, MOTOR_PWM_MAX_DUTY);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 设置PWM占空比 / Set PWM duty cycle
    esp_err_t ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置PWM占空比失败 / Failed to set PWM duty cycle");
        return ret;
    }
    
    // 更新PWM / Update PWM
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "更新PWM失败 / Failed to update PWM");
        return ret;
    }
    
    // 更新状态 / Update state
    g_motor_state.duty_cycle = duty_cycle;
    g_motor_state.speed_percentage = (float)duty_cycle / MOTOR_PWM_MAX_DUTY * 100.0f;

    // 节流日志：每5秒打印一次 / Throttle log: print once every 5 seconds
    {
        static TickType_t s_last_log_ticks = 0;
        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_log_ticks) >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG, "电机速度: %lu/%d (%.1f%%)", duty_cycle, MOTOR_PWM_MAX_DUTY, g_motor_state.speed_percentage);
            s_last_log_ticks = now;
        }
    }

    return ESP_OK;
}

/**
 * @brief 设置电机速度 (百分比)
 */
esp_err_t motor_set_speed_percentage(float percentage)
{
    // 检查百分比范围 / Check percentage range
    if (percentage < 0.0f || percentage > 100.0f) {
        ESP_LOGE(TAG, "速度百分比超出范围: %.1f%% / Speed percentage out of range: %.1f%%", percentage);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 转换为占空比 / Convert to duty cycle
    uint32_t duty_cycle = (uint32_t)(percentage / 100.0f * MOTOR_PWM_MAX_DUTY);
    
    return motor_set_speed(duty_cycle);
}

/**
 * @brief 启动电机
 */
esp_err_t motor_start(void)
{
    if (g_motor_state.is_running) {
        ESP_LOGW(TAG, "电机已在运行 / Motor is already running");
        return ESP_OK;
    }
    
    // 设置方向为正转 / Set direction to forward
    esp_err_t ret = motor_set_direction(MOTOR_FORWARD);
    if (ret != ESP_OK) {
        return ret;
    }
    
    g_motor_state.is_running = true;
    ESP_LOGI(TAG, "电机已启动 / Motor started");
    
    return ESP_OK;
}

/**
 * @brief 停止电机
 */
esp_err_t motor_stop(void)
{
    if (!g_motor_state.is_running) {
        ESP_LOGW(TAG, "电机已停止 / Motor is already stopped");
        return ESP_OK;
    }
    
    // 设置方向为停止 / Set direction to stop
    esp_err_t ret = motor_set_direction(MOTOR_STOP);
    if (ret != ESP_OK) {
        return ret;
    }
    
    g_motor_state.is_running = false;
    ESP_LOGI(TAG, "电机已停止 / Motor stopped");
    
    return ESP_OK;
}

/**
 * @brief 获取电机当前状态
 */
esp_err_t motor_get_state(motor_state_t *state)
{
    if (state == NULL) {
        ESP_LOGE(TAG, "状态指针为空 / State pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(state, &g_motor_state, sizeof(motor_state_t));
    return ESP_OK;
}

/**
 * @brief 电机紧急停止
 */
esp_err_t motor_emergency_stop(void)
{
    ESP_LOGW(TAG, "紧急停止电机! / Emergency stop motor!");

    // 立即停止电机 / Immediately stop motor
    motor_set_direction(MOTOR_STOP);
    motor_set_speed(0);

    g_motor_state.is_running = false;
    g_motor_state.duty_cycle = 0;
    g_motor_state.speed_percentage = 0.0f;

    return ESP_OK;
}


/* ========== 电机2控制实现 / Motor 2 Control Implementation ========== */

// 电机2状态全局变量 / Global motor 2 state variable
static motor_state_t g_motor2_state = {
    .is_running = false,
    .direction = MOTOR_STOP,
    .duty_cycle = 0,
    .speed_percentage = 0.0f
};

// LEDC PWM配置 for Motor 2
#define LEDC2_TIMER              LEDC_TIMER_1
#define LEDC2_MODE               LEDC_LOW_SPEED_MODE
#define LEDC2_CHANNEL            LEDC_CHANNEL_1

/**
 * @brief 初始化电机2驱动器
 */
esp_err_t motor2_driver_init(void)
{
    ESP_LOGI(TAG, "初始化电机2驱动器... / Initializing motor 2 driver...");

    // 配置方向控制引脚 (IN1, IN2) / Configure direction control pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR2_IN1_GPIO) | (1ULL << MOTOR2_IN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机2方向引脚配置失败 / Motor 2 direction pins configuration failed");
        return ret;
    }

    // 初始化方向引脚为停止状态 / Initialize direction pins to stop state
    gpio_set_level(MOTOR2_IN1_GPIO, 0);
    gpio_set_level(MOTOR2_IN2_GPIO, 0);

    // 配置LEDC定时器 / Configure LEDC timer
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC2_MODE;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.freq_hz = LEDC_FREQUENCY;
    ledc_timer.timer_num = LEDC2_TIMER;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机2 LEDC定时器配置失败 / Motor 2 LEDC timer configuration failed");
        return ret;
    }

    // 配置LEDC通道 / Configure LEDC channel
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num = MOTOR2_PWM_GPIO;
    ledc_channel.speed_mode = LEDC2_MODE;
    ledc_channel.channel = LEDC2_CHANNEL;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel = LEDC2_TIMER;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机2 LEDC通道配置失败 / Motor 2 LEDC channel configuration failed");
        return ret;
    }

    ESP_LOGI(TAG, "电机2驱动器初始化成功 / Motor 2 driver initialized successfully");
    ESP_LOGI(TAG, "  IN1: GPIO %d", MOTOR2_IN1_GPIO);
    ESP_LOGI(TAG, "  IN2: GPIO %d", MOTOR2_IN2_GPIO);
    ESP_LOGI(TAG, "  PWM: GPIO %d (频率: %d Hz)", MOTOR2_PWM_GPIO, LEDC_FREQUENCY);

    return ESP_OK;
}

/**
 * @brief 设置电机2方向
 *
 * 注意：电机2（左轮）的方向与电机1相反，因为两个电机面对面安装
 * Note: Motor 2 (left wheel) direction is reversed from Motor 1 due to face-to-face mounting
 */
esp_err_t motor2_set_direction(motor_direction_t direction)
{
    // 安全检查：如果要改变方向（不是停止），必须先确保电机已停止
    // Safety check: If changing direction (not stopping), motor must be stopped first
    if (direction != MOTOR_STOP && direction != g_motor2_state.direction) {
        if (g_motor2_state.is_running && g_motor2_state.duty_cycle > 0) {
            ESP_LOGE(TAG, "安全错误：电机2必须先停止才能改变方向！");
            ESP_LOGE(TAG, "Safety Error: Motor 2 must be stopped before changing direction!");
            return ESP_FAIL;
        }
    }

    switch (direction) {
        case MOTOR_FORWARD:
            // 左轮正转需要反向信号 / Left wheel forward needs reversed signal
            gpio_set_level(MOTOR2_IN1_GPIO, 0);
            gpio_set_level(MOTOR2_IN2_GPIO, 1);
            g_motor2_state.direction = MOTOR_FORWARD;
            ESP_LOGI(TAG, "电机2方向: 正转 / Motor 2 direction: FORWARD");
            break;

        case MOTOR_BACKWARD:
            // 左轮反转需要正向信号 / Left wheel backward needs forward signal
            gpio_set_level(MOTOR2_IN1_GPIO, 1);
            gpio_set_level(MOTOR2_IN2_GPIO, 0);
            g_motor2_state.direction = MOTOR_BACKWARD;
            ESP_LOGI(TAG, "电机2方向: 反转 / Motor 2 direction: BACKWARD");
            break;

        case MOTOR_STOP:
        default:
            gpio_set_level(MOTOR2_IN1_GPIO, 0);
            gpio_set_level(MOTOR2_IN2_GPIO, 0);
            g_motor2_state.direction = MOTOR_STOP;
            ESP_LOGI(TAG, "电机2方向: 停止 / Motor 2 direction: STOP");
            break;
    }

    return ESP_OK;
}

/**
 * @brief 设置电机2速度
 */
esp_err_t motor2_set_speed(uint32_t duty_cycle)
{
    if (duty_cycle > MOTOR_PWM_MAX_DUTY) {
        ESP_LOGW(TAG, "电机2占空比超出范围，限制到最大值 / Motor 2 duty cycle out of range, clamping to max");
        duty_cycle = MOTOR_PWM_MAX_DUTY;
    }

    esp_err_t ret = ledc_set_duty(LEDC2_MODE, LEDC2_CHANNEL, duty_cycle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机2设置PWM占空比失败 / Motor 2 set PWM duty failed");
        return ret;
    }

    ret = ledc_update_duty(LEDC2_MODE, LEDC2_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机2更新PWM占空比失败 / Motor 2 update PWM duty failed");
        return ret;
    }

    g_motor2_state.duty_cycle = duty_cycle;
    g_motor2_state.speed_percentage = ((float)duty_cycle / (float)MOTOR_PWM_MAX_DUTY) * 100.0f;

    return ESP_OK;
}

/**
 * @brief 启动电机2
 */
esp_err_t motor2_start(void)
{
    g_motor2_state.is_running = true;
    return ESP_OK;
}

/**
 * @brief 停止电机2
 */
esp_err_t motor2_stop(void)
{
    motor2_set_direction(MOTOR_STOP);
    g_motor2_state.is_running = false;
    return ESP_OK;
}

/**
 * @brief 获取电机2状态
 */
esp_err_t motor2_get_state(motor_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(state, &g_motor2_state, sizeof(motor_state_t));
    return ESP_OK;
}

