/**
 * @file pid_v2.cpp
 * @brief PID控制器 V2 - 纯算法实现
 * PID Controller V2 - Pure algorithm implementation
 * 
 * 这是一个标准的PID控制器，不包含任何硬件相关代码
 * This is a standard PID controller without any hardware-specific code
 */

#include "include/pid_v2.h"
#include <string.h>

static inline float clamp_float(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void pid_v2_init(pid_v2_t *pid, float kp, float ki, float kd, float out_min, float out_max) {
    if (!pid) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    
    pid->integrator = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = false;
}

void pid_v2_reset(pid_v2_t *pid) {
    if (!pid) return;
    
    pid->integrator = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = false;
}

float pid_v2_compute(pid_v2_t *pid, float setpoint, float measurement, float dt) {
    if (!pid) return 0.0f;
    if (dt <= 0.0f) dt = 0.001f;  // 防止除零
    
    // 计算误差
    float error = setpoint - measurement;
    
    // P项：比例控制
    float p_term = pid->kp * error;
    
    // I项：积分控制（带抗饱和）
    pid->integrator += pid->ki * error * dt;
    
    // 积分项限幅到输出范围的80%，留20%给P和D项
    float i_max = (pid->out_max - pid->out_min) * 0.8f;
    pid->integrator = clamp_float(pid->integrator, -i_max, i_max);
    float i_term = pid->integrator;
    
    // D项：微分控制（对测量值微分，减少噪声影响）
    float d_term = 0.0f;
    if (pid->initialized) {
        float derivative = (measurement - pid->prev_measurement) / dt;
        d_term = -pid->kd * derivative;  // 负号：对测量值微分
    } else {
        pid->initialized = true;
    }
    pid->prev_measurement = measurement;
    
    // 组合输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    output = clamp_float(output, pid->out_min, pid->out_max);
    
    return output;
}

void pid_v2_get_state(pid_v2_t *pid, pid_v2_state_t *state) {
    if (!pid || !state) return;
    
    state->kp = pid->kp;
    state->ki = pid->ki;
    state->kd = pid->kd;
    state->integrator = pid->integrator;
    state->prev_measurement = pid->prev_measurement;
    state->out_min = pid->out_min;
    state->out_max = pid->out_max;
}

