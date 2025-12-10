/**
 * @file pid_v2.h
 * @brief PID控制器 V2 - 头文件
 * PID Controller V2 - Header
 */

#ifndef PID_V2_H
#define PID_V2_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID控制器结构体
 */
typedef struct {
    // PID参数
    float kp;              // 比例增益
    float ki;              // 积分增益
    float kd;              // 微分增益
    
    // 内部状态
    float integrator;      // 积分累加器
    float prev_measurement;// 上次测量值（用于微分）
    bool initialized;      // 是否已初始化
    
    // 输出限幅
    float out_min;         // 输出最小值
    float out_max;         // 输出最大值
} pid_v2_t;

/**
 * @brief PID状态（用于调试）
 */
typedef struct {
    float kp, ki, kd;
    float integrator;
    float prev_measurement;
    float out_min, out_max;
} pid_v2_state_t;

/**
 * @brief 初始化PID控制器
 * 
 * @param pid PID控制器指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param out_min 输出最小值
 * @param out_max 输出最大值
 */
void pid_v2_init(pid_v2_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief 重置PID控制器
 * 
 * @param pid PID控制器指针
 */
void pid_v2_reset(pid_v2_t *pid);

/**
 * @brief 计算PID输出
 * 
 * @param pid PID控制器指针
 * @param setpoint 目标值
 * @param measurement 测量值
 * @param dt 时间间隔（秒）
 * @return PID输出值
 */
float pid_v2_compute(pid_v2_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief 获取PID状态（用于调试）
 * 
 * @param pid PID控制器指针
 * @param state 状态结构体指针
 */
void pid_v2_get_state(pid_v2_t *pid, pid_v2_state_t *state);

#ifdef __cplusplus
}
#endif

#endif // PID_V2_H

