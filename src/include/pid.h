/**
 * @file pid.h
 * @brief PID 控制器接口 / PID Controller Interface
 *
 * 提供标准 PID(比例-积分-微分) 控制器的实现和API
 * Provides a standard PID (Proportional-Integral-Derivative) controller implementation and APIs
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 控制器数据结构 / PID Controller structure
 */
typedef struct {
    // 增益参数 / Gains
    float kp;  /**< 比例增益 / Proportional gain */
    float ki;  /**< 积分增益 / Integral gain */
    float kd;  /**< 微分增益 / Derivative gain */

    // 内部状态 / Internal states
    float integrator;         /**< 积分项累计值 / Integral accumulator */
    float prev_error;         /**< 上次误差 / Previous error */
    float prev_measurement;   /**< 上次测量值 / Previous measurement (for derivative on measurement) */
    bool initialized;         /**< 是否已初始化微分起点 / Derivative init flag */

    // 输出限幅 / Output limits
    float out_min;            /**< 最小输出 / Minimum output */
    float out_max;            /**< 最大输出 / Maximum output */
} pid_controller_t;

/**
 * @brief 初始化 PID 控制器 / Initialize PID controller
 * @param pid  控制器句柄 / Controller handle
 * @param kp   比例增益 / Proportional gain
 * @param ki   积分增益 / Integral gain
 * @param kd   微分增益 / Derivative gain
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief 运行时调整 PID 参数 / Adjust PID tunings at runtime
 */
void pid_set_tunings(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置输出限幅 / Set output limits
 * @param pid      控制器句柄 / Controller handle
 * @param out_min  最小输出 / Minimum output
 * @param out_max  最大输出 / Maximum output
 */
void pid_set_limits(pid_controller_t *pid, float out_min, float out_max);

/**
 * @brief 清零状态(积分/历史误差) / Reset integral and history
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief 计算 PID 输出 / Compute PID output
 *
 * @param pid          控制器句柄 / Controller handle
 * @param setpoint     目标值(例如 RPM) / Target setpoint (e.g., RPM)
 * @param measurement  实际测量值(例如 RPM) / Current measurement (e.g., RPM)
 * @param dt_seconds   采样周期(秒) / Sample time (seconds)
 * @return PID 输出，未舍入的连续值 / PID output (continuous)
 */
float pid_compute(pid_controller_t *pid, float setpoint, float measurement, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif // PID_H

