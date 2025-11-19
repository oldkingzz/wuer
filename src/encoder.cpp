/**
 * @file encoder.cpp
 * @brief 编码器实现 / Encoder Implementation
 *
 * 实现正交编码器的脉冲计数和速度测量功能
 * Implements pulse counting and speed measurement for quadrature encoder
 */

#include <string.h>
#include "Arduino.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/encoder.h"
#include "include/gpio_config.h"

static const char *TAG = "ENCODER";

// 编码器状态变量 / Encoder state variables
static volatile int32_t g_pulse_count = 0;      // 脉冲计数 / Pulse count
static volatile int32_t g_total_count = 0;      // 总计数 (4倍频) / Total count (4x)
static volatile int32_t g_last_count = 0;       // 上次计数 / Last count
static volatile uint32_t g_last_time_us = 0;    // 上次时间 (微秒) / Last time (us)
static volatile float g_current_rpm = 0.0f;     // 当前转速 / Current RPM

// 编码器A/B相上次状态 / Last state of encoder A/B channels
static volatile uint8_t g_last_state = 0;

/**
 * @brief 编码器中断服务程序 (ISR)
 * Encoder Interrupt Service Routine
 * 
 * 使用4倍频解码算法
 * Uses 4x decoding algorithm
 */
static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    // 读取当前A/B相状态 / Read current A/B channel states
    uint8_t a_state = gpio_get_level(ENCODER_A_GPIO);
    uint8_t b_state = gpio_get_level(ENCODER_B_GPIO);
    uint8_t current_state = (a_state << 1) | b_state;
    
    // 4倍频正交解码状态机 / 4x quadrature decoding state machine
    // 状态转换表 / State transition table
    // 正转序列: 00 -> 01 -> 11 -> 10 -> 00 (顺时针)
    // Forward: 00 -> 01 -> 11 -> 10 -> 00 (Clockwise)
    // 反转序列: 00 -> 10 -> 11 -> 01 -> 00 (逆时针)
    // Backward: 00 -> 10 -> 11 -> 01 -> 00 (Counter-clockwise)
    
    uint8_t state_change = (g_last_state << 2) | current_state;
    
    switch (state_change) {
        // 正转 / Forward
        case 0b0001:  // 00 -> 01
        case 0b0111:  // 01 -> 11
        case 0b1110:  // 11 -> 10
        case 0b1000:  // 10 -> 00
            g_total_count++;
            break;
            
        // 反转 / Backward
        case 0b0010:  // 00 -> 10
        case 0b1011:  // 10 -> 11
        case 0b1101:  // 11 -> 01
        case 0b0100:  // 01 -> 00
            g_total_count--;
            break;
            
        // 无效转换或噪声 / Invalid transition or noise
        default:
            break;
    }
    
    g_last_state = current_state;
    
    // 每4个计数增加一个脉冲 / Increment pulse every 4 counts
    g_pulse_count = g_total_count / 4;
}

/**
 * @brief 初始化编码器
 */
esp_err_t encoder_init(void)
{
    ESP_LOGI(TAG, "初始化编码器... / Initializing encoder...");
    
    // 配置编码器A相引脚 / Configure encoder A channel pin
    gpio_config_t io_conf_a = {
        .pin_bit_mask = (1ULL << ENCODER_A_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE  // 任意边沿触发 / Trigger on any edge
    };
    
    esp_err_t ret = gpio_config(&io_conf_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器A相GPIO配置失败 / Encoder A channel GPIO config failed");
        return ret;
    }
    
    // 配置编码器B相引脚 / Configure encoder B channel pin
    gpio_config_t io_conf_b = {
        .pin_bit_mask = (1ULL << ENCODER_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE  // 任意边沿触发 / Trigger on any edge
    };
    
    ret = gpio_config(&io_conf_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器B相GPIO配置失败 / Encoder B channel GPIO config failed");
        return ret;
    }
    
    // 读取初始状态 / Read initial state
    uint8_t a_state = gpio_get_level(ENCODER_A_GPIO);
    uint8_t b_state = gpio_get_level(ENCODER_B_GPIO);
    g_last_state = (a_state << 1) | b_state;
    
    // 安装GPIO中断服务 / Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE表示已经安装 / ESP_ERR_INVALID_STATE means already installed
        ESP_LOGE(TAG, "GPIO中断服务安装失败 / GPIO ISR service installation failed");
        return ret;
    }
    
    // 添加中断处理程序 / Add ISR handlers
    ret = gpio_isr_handler_add(ENCODER_A_GPIO, encoder_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器A相中断处理程序添加失败 / Encoder A ISR handler add failed");
        return ret;
    }
    
    ret = gpio_isr_handler_add(ENCODER_B_GPIO, encoder_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器B相中断处理程序添加失败 / Encoder B ISR handler add failed");
        gpio_isr_handler_remove(ENCODER_A_GPIO);
        return ret;
    }
    
    // 初始化时间戳 / Initialize timestamp
    g_last_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "编码器初始化成功 / Encoder initialized successfully");
    ESP_LOGI(TAG, "编码器参数 - Base CPR: %d, 4x: %d, Gear: %d, Total CPR: %d",
             ENCODER_BASE_CPR, ENCODER_CPR_4X, MOTOR_GEAR_RATIO, ENCODER_CPR);

    return ESP_OK;
}

/**
 * @brief 重置编码器计数
 */
esp_err_t encoder_reset(void)
{
    g_pulse_count = 0;
    g_total_count = 0;
    g_last_count = 0;
    g_current_rpm = 0.0f;
    g_last_time_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "编码器计数已重置 / Encoder count reset");
    return ESP_OK;
}

/**
 * @brief 获取编码器脉冲计数
 */
int32_t encoder_get_count(void)
{
    return g_pulse_count;
}

/**
 * @brief 获取编码器总计数 (4倍频)
 */
int32_t encoder_get_total_count(void)
{
    return g_total_count;
}

/**
 * @brief 获取转数
 */
float encoder_get_revolutions(void)
{
    return (float)g_pulse_count / ENCODER_CPR;
}

/**
 * @brief 获取转速 (RPM)
 */
float encoder_get_rpm(void)
{
    return g_current_rpm;
}

/**
 * @brief 更新转速计算
 */
esp_err_t encoder_update_speed(void)
{
    uint32_t current_time_us = esp_timer_get_time();
    int32_t current_count = g_pulse_count;
    
    // 计算时间差 (秒) / Calculate time difference (seconds)
    float time_diff_s = (current_time_us - g_last_time_us) / 1000000.0f;
    
    if (time_diff_s > 0.0f) {
        // 计算脉冲差 / Calculate pulse difference
        int32_t count_diff = current_count - g_last_count;

        // 计算转速 (RPM) / Calculate speed (RPM)
        // RPM = (脉冲差 / CPR) / 时间差(秒) * 60
        // RPM = (pulse_diff / CPR) / time_diff(s) * 60
        g_current_rpm = ((float)count_diff / ENCODER_CPR) / time_diff_s * 60.0f;
        
        // 更新上次值 / Update last values
        g_last_count = current_count;
        g_last_time_us = current_time_us;
    }
    
    return ESP_OK;
}

/**
 * @brief 获取编码器完整数据
 */
esp_err_t encoder_get_data(encoder_data_t *data)
{
    if (data == NULL) {
        ESP_LOGE(TAG, "数据指针为空 / Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    data->pulse_count = g_pulse_count;
    data->total_count = g_total_count;
    data->revolutions = encoder_get_revolutions();
    data->rpm = g_current_rpm;
    data->last_update_time = (uint32_t)(g_last_time_us / 1000);
    
    return ESP_OK;
}

/**
 * @brief 打印编码器信息
 */
void encoder_print_info(void)
{
    encoder_data_t data;
    encoder_get_data(&data);

    ESP_LOGI(TAG, "========== 编码器信息 / Encoder Info ==========");
    ESP_LOGI(TAG, "脉冲计数 / Pulse Count: %ld", data.pulse_count);
    ESP_LOGI(TAG, "总计数 (4x) / Total Count (4x): %ld", data.total_count);
    ESP_LOGI(TAG, "转数 / Revolutions: %.2f", data.revolutions);
    ESP_LOGI(TAG, "转速 / RPM: %.2f", data.rpm);
    ESP_LOGI(TAG, "=============================================");
}


/* ========== 编码器2实现 / Encoder 2 Implementation ========== */

// 编码器2状态变量 / Encoder 2 state variables
static volatile int32_t g_pulse_count2 = 0;
static volatile int32_t g_total_count2 = 0;
static volatile int32_t g_last_count2 = 0;
static volatile uint32_t g_last_time_us2 = 0;
static volatile float g_current_rpm2 = 0.0f;
static volatile uint8_t g_last_state2 = 0;

/**
 * @brief 编码器2中断服务程序
 */
static void IRAM_ATTR encoder2_isr_handler(void* arg)
{
    uint8_t a_state = gpio_get_level(ENCODER2_A_GPIO);
    uint8_t b_state = gpio_get_level(ENCODER2_B_GPIO);
    uint8_t current_state = (a_state << 1) | b_state;

    uint8_t combined = (g_last_state2 << 2) | current_state;

    switch (combined) {
        case 0b0001: case 0b0111: case 0b1110: case 0b1000:
            g_total_count2++;
            break;
        case 0b0010: case 0b1011: case 0b1101: case 0b0100:
            g_total_count2--;
            break;
        default:
            break;
    }

    if (g_total_count2 % 4 == 0) {
        if (combined == 0b0001 || combined == 0b0111 || combined == 0b1110 || combined == 0b1000) {
            g_pulse_count2++;
        } else if (combined == 0b0010 || combined == 0b1011 || combined == 0b1101 || combined == 0b0100) {
            g_pulse_count2--;
        }
    }

    g_last_state2 = current_state;
}

/**
 * @brief 初始化编码器2
 */
esp_err_t encoder2_init(void)
{
    ESP_LOGI(TAG, "初始化编码器2... / Initializing encoder 2...");
    Serial.println("  [encoder2_init] Starting...");
    Serial.flush();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER2_A_GPIO) | (1ULL << ENCODER2_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    Serial.println("  [encoder2_init] Configuring GPIO...");
    Serial.flush();
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器2 GPIO配置失败 / Encoder 2 GPIO configuration failed");
        Serial.println("  [encoder2_init] GPIO config failed!");
        return ret;
    }
    Serial.println("  [encoder2_init] GPIO configured");
    Serial.flush();

    // GPIO ISR服务已经在encoder_init()中安装，这里不需要再次安装
    // GPIO ISR service already installed in encoder_init(), no need to install again

    Serial.println("  [encoder2_init] Adding ISR for channel A...");
    Serial.flush();
    ret = gpio_isr_handler_add(ENCODER2_A_GPIO, encoder2_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器2 A相ISR添加失败 / Encoder 2 channel A ISR add failed");
        Serial.println("  [encoder2_init] ISR A failed!");
        return ret;
    }
    Serial.println("  [encoder2_init] ISR A added");
    Serial.flush();

    Serial.println("  [encoder2_init] Adding ISR for channel B...");
    Serial.flush();
    ret = gpio_isr_handler_add(ENCODER2_B_GPIO, encoder2_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器2 B相ISR添加失败 / Encoder 2 channel B ISR add failed");
        Serial.println("  [encoder2_init] ISR B failed!");
        gpio_isr_handler_remove(ENCODER2_A_GPIO);
        return ret;
    }
    Serial.println("  [encoder2_init] ISR B added");
    Serial.flush();

    uint8_t a_state = gpio_get_level(ENCODER2_A_GPIO);
    uint8_t b_state = gpio_get_level(ENCODER2_B_GPIO);
    g_last_state2 = (a_state << 1) | b_state;

    g_last_time_us2 = (uint32_t)esp_timer_get_time();

    ESP_LOGI(TAG, "编码器2初始化成功 / Encoder 2 initialized successfully");
    ESP_LOGI(TAG, "  A相: GPIO %d", ENCODER2_A_GPIO);
    ESP_LOGI(TAG, "  B相: GPIO %d", ENCODER2_B_GPIO);

    Serial.println("  [encoder2_init] Complete!");
    Serial.flush();

    return ESP_OK;
}

/**
 * @brief 重置编码器2
 */
esp_err_t encoder2_reset(void)
{
    g_pulse_count2 = 0;
    g_total_count2 = 0;
    g_last_count2 = 0;
    g_current_rpm2 = 0.0f;
    g_last_time_us2 = (uint32_t)esp_timer_get_time();

    ESP_LOGI(TAG, "编码器2已重置 / Encoder 2 reset");
    return ESP_OK;
}

/**
 * @brief 获取编码器2脉冲计数
 */
int32_t encoder2_get_count(void)
{
    return g_pulse_count2;
}

/**
 * @brief 获取编码器2转速
 */
float encoder2_get_rpm(void)
{
    return g_current_rpm2;
}

/**
 * @brief 获取编码器2完整数据
 */
esp_err_t encoder2_get_data(encoder_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    data->pulse_count = g_pulse_count2;
    data->total_count = g_total_count2;
    data->revolutions = (float)g_pulse_count2 / (float)ENCODER_CPR;
    data->rpm = g_current_rpm2;
    data->last_update_time = (uint32_t)(g_last_time_us2 / 1000);

    return ESP_OK;
}

/**
 * @brief 更新编码器2转速
 */
esp_err_t encoder2_update_speed(void)
{
    uint32_t current_time_us = (uint32_t)esp_timer_get_time();
    uint32_t delta_time_us = current_time_us - g_last_time_us2;

    if (delta_time_us == 0) {
        return ESP_OK;
    }

    int32_t delta_count = g_pulse_count2 - g_last_count2;
    float delta_time_s = (float)delta_time_us / 1000000.0f;
    float delta_revolutions = (float)delta_count / (float)ENCODER_CPR;
    g_current_rpm2 = (delta_revolutions / delta_time_s) * 60.0f;

    g_last_count2 = g_pulse_count2;
    g_last_time_us2 = current_time_us;

    return ESP_OK;
}

