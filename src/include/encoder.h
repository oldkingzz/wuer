/**
 * @file encoder.h
 * @brief 编码器接口 / Encoder Interface
 * 
 * 本模块提供正交编码器的脉冲计数和速度测量功能
 * This module provides pulse counting and speed measurement for quadrature encoder
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief 编码器数据结构
 * Encoder Data Structure
 */
typedef struct {
    int32_t pulse_count;        /**< 脉冲计数 / Pulse count */
    int32_t total_count;        /**< 总计数 (4倍频) / Total count (4x decoding) */
    float revolutions;          /**< 转数 / Number of revolutions */
    float rpm;                  /**< 转速 (RPM) / Speed in RPM */
    uint32_t last_update_time;  /**< 上次更新时间 (ms) / Last update time (ms) */
} encoder_data_t;

/**
 * @brief 初始化编码器
 * Initialize Encoder
 * 
 * 配置GPIO引脚和中断处理
 * Configures GPIO pins and interrupt handling
 * 
 * @return 
 *     - ESP_OK: 成功 / Success
 *     - ESP_FAIL: 失败 / Failure
 */
esp_err_t encoder_init(void);

/**
 * @brief 重置编码器计数
 * Reset Encoder Count
 * 
 * 将所有计数器清零
 * Clears all counters to zero
 * 
 * @return 
 *     - ESP_OK: 成功 / Success
 */
esp_err_t encoder_reset(void);

/**
 * @brief 获取编码器脉冲计数
 * Get Encoder Pulse Count
 * 
 * @return 当前脉冲计数 / Current pulse count
 */
int32_t encoder_get_count(void);

/**
 * @brief 获取编码器总计数 (4倍频)
 * Get Encoder Total Count (4x decoding)
 * 
 * @return 当前总计数 / Current total count
 */
int32_t encoder_get_total_count(void);

/**
 * @brief 获取转数
 * Get Number of Revolutions
 * 
 * @return 转数 / Number of revolutions
 */
float encoder_get_revolutions(void);

/**
 * @brief 获取转速 (RPM)
 * Get Speed in RPM
 * 
 * 基于最近的脉冲计数计算转速
 * Calculates speed based on recent pulse counts
 * 
 * @return 转速 (RPM) / Speed in RPM
 */
float encoder_get_rpm(void);

/**
 * @brief 获取编码器完整数据
 * Get Complete Encoder Data
 * 
 * @param data 指向数据结构的指针 / Pointer to data structure
 * @return 
 *     - ESP_OK: 成功 / Success
 *     - ESP_ERR_INVALID_ARG: 空指针 / Null pointer
 */
esp_err_t encoder_get_data(encoder_data_t *data);

/**
 * @brief 更新转速计算
 * Update Speed Calculation
 * 
 * 应在定时器中周期性调用以更新RPM计算
 * Should be called periodically in a timer to update RPM calculation
 * 
 * @return 
 *     - ESP_OK: 成功 / Success
 */
esp_err_t encoder_update_speed(void);

/**
 * @brief 打印编码器信息
 * Print Encoder Information
 *
 * 输出当前编码器状态到日志
 * Outputs current encoder state to log
 */
void encoder_print_info(void);


/* ========== 编码器2控制函数 / Encoder 2 Control Functions ========== */

/**
 * @brief 初始化编码器2
 * Initialize Encoder 2
 *
 * @return
 *     - ESP_OK: 成功 / Success
 *     - ESP_FAIL: 失败 / Failure
 */
esp_err_t encoder2_init(void);

/**
 * @brief 重置编码器2计数
 * Reset Encoder 2 Count
 *
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t encoder2_reset(void);

/**
 * @brief 获取编码器2脉冲计数
 * Get Encoder 2 Pulse Count
 *
 * @return 当前脉冲计数 / Current pulse count
 */
int32_t encoder2_get_count(void);

/**
 * @brief 获取编码器2转速 (RPM)
 * Get Encoder 2 Speed in RPM
 *
 * @return 转速 (RPM) / Speed in RPM
 */
float encoder2_get_rpm(void);

/**
 * @brief 获取编码器2完整数据
 * Get Complete Encoder 2 Data
 *
 * @param data 指向数据结构的指针 / Pointer to data structure
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t encoder2_get_data(encoder_data_t *data);

/**
 * @brief 更新编码器2转速计算
 * Update Encoder 2 Speed Calculation
 *
 * @return
 *     - ESP_OK: 成功 / Success
 */
esp_err_t encoder2_update_speed(void);

#endif // ENCODER_H

