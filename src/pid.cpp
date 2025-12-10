/**
 * @file pid.c
 * @brief PID 控制器实现 / PID Controller implementation
 */

#include "include/pid.h"
#include <math.h>

// 安全夹紧 / Safe clamp
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd)
{
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = false;
    // 默认输出范围 0..1023（可用 pid_set_limits 调整） / default output range 0..1023
    pid->out_min = 0.0f;
    pid->out_max = 1023.0f;
}

void pid_set_tunings(pid_controller_t *pid, float kp, float ki, float kd)
{
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_limits(pid_controller_t *pid, float out_min, float out_max)
{
    if (!pid) return;
    // 确保 min <= max / ensure min <= max
    if (out_min > out_max) {
        float t = out_min; out_min = out_max; out_max = t;
    }
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void pid_reset(pid_controller_t *pid)
{
    if (!pid) return;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = false;
}

float pid_compute(pid_controller_t *pid, float setpoint, float measurement, float dt_seconds)
{
    if (!pid) return 0.0f;
    if (dt_seconds <= 0.0f) dt_seconds = 1e-3f; // 防止除零 / avoid div by zero

    // 误差 / error
    float error = setpoint - measurement;

    // 比例项 / proportional term
    float p_term = pid->kp * error;

    // 积分项（简单矩形积分）/ integral term (rectangle rule)
    pid->integrator += pid->ki * error * dt_seconds;

    // 抗积分饱和：积分项限幅到输出范围的80%，留空间给P和D项
    // anti-windup: clamp integrator to 80% of output range, leave room for P and D
    float i_max = (pid->out_max - pid->out_min) * 0.8f;
    float i_term = clampf(pid->integrator, -i_max, i_max);
    pid->integrator = i_term;

    // 微分项（对测量微分，更抗噪）/ derivative on measurement (less noise)
    float d_term = 0.0f;
    if (pid->initialized) {
        float d_meas = (measurement - pid->prev_measurement) / dt_seconds;
        d_term = -pid->kd * d_meas; // 对测量微分取负 / negative of measurement derivative
    } else {
        d_term = 0.0f;
        pid->initialized = true;
    }

    // 组合输出 / combine
    float output = p_term + i_term + d_term;

    // 输出限幅 / clamp
    output = clampf(output, pid->out_min, pid->out_max);

    // 更新历史 / update history
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
}

