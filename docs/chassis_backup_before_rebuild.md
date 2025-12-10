# Chassis System Backup - Before PID Rebuild

**Date**: 2025-12-09  
**Reason**: User swapped motor power connections (IN1 ↔ IN2), causing only left wheel to turn on forward command. Rebuilding PID from scratch.

---

## Hardware Configuration

### Motor Wiring (AFTER SWAP)
- **Motor 1 (Right Wheel)**: IN1 and IN2 **SWAPPED**
- **Motor 2 (Left Wheel)**: IN1 and IN2 **SWAPPED**

### Original Wiring (BEFORE SWAP)
- **Motor 1 (Right Wheel)**: Normal
- **Motor 2 (Left Wheel)**: Reversed (negative encoder reading)

### Physical Layout
```
        Front
         ↑
    [Left] [Right]
    Motor2  Motor1
```

### Chassis Parameters
```cpp
#define CHASSIS_WHEEL_BASE_M        0.15f    // 轮距 15cm
#define CHASSIS_WHEEL_DIAMETER_M    0.065f   // 轮径 6.5cm
#define CHASSIS_WHEEL_RADIUS_M      0.0325f  // 轮半径 3.25cm
#define CHASSIS_MAX_LINEAR_VELOCITY 0.5f     // 最大线速度 0.5 m/s
#define CHASSIS_MAX_ANGULAR_VELOCITY 2.0f    // 最大角速度 2.0 rad/s
#define CHASSIS_MAX_WHEEL_RPM       150.0f   // 最大轮速 150 RPM
```

### Encoder Configuration
```cpp
// Motor 1 (Right wheel)
#define ENCODER_PULSES_PER_REV  3200  // 4x decoding
float right_actual_rpm = fabsf(encoder_get_rpm());

// Motor 2 (Left wheel) - REVERSED WIRING
#define ENCODER2_PULSES_PER_REV 3200  // 4x decoding
float left_actual_rpm = fabsf(-encoder2_get_rpm());  // Negate due to reversed wiring
```

---

## Current PID Parameters (FINAL VERSION)

### PID Gains
```cpp
// Left wheel
Kp = 10.0
Ki = 2.5
Kd = 0.2

// Right wheel
Kp = 10.0
Ki = 2.5
Kd = 0.2
```

### PID Limits
```cpp
Output range: [0, 1023]  // PWM duty cycle
Integrator limit: 80% of output range = 818
Dead zone: ±3 RPM (no PWM adjustment when error < 3 RPM)
```

### PID Control Loop
```cpp
Period: 50ms (20 Hz)
dt = 0.05 seconds
```

---

## Chassis API (Public Interface)

### Initialization
```cpp
esp_err_t chassis_init(void);
```

### Velocity Control
```cpp
esp_err_t chassis_set_velocity(float linear_velocity, float angular_velocity);
// linear_velocity: m/s, range [-0.5, 0.5]
// angular_velocity: rad/s, range [-2.0, 2.0]
```

### State Query
```cpp
esp_err_t chassis_get_velocity(chassis_velocity_t *velocity);

typedef struct {
    float linear_velocity;   // m/s
    float angular_velocity;  // rad/s
    float left_wheel_rpm;    // RPM (can be negative)
    float right_wheel_rpm;   // RPM (can be negative)
    bool is_moving;
} chassis_velocity_t;
```

### Stop
```cpp
esp_err_t chassis_stop(void);
```

---

## Differential Drive Kinematics

### Inverse Kinematics (Velocity → Wheel Speed)
```cpp
v_left = v - ω * L / 2
v_right = v + ω * L / 2

where:
  v = linear velocity (m/s)
  ω = angular velocity (rad/s)
  L = wheel base (0.15 m)
```

### Forward Kinematics (Wheel Speed → Velocity)
```cpp
v = (v_left + v_right) / 2
ω = (v_right - v_left) / L
```

### Velocity to RPM Conversion
```cpp
RPM = (velocity_m_s / (π * wheel_diameter)) * 60
    = (velocity_m_s / (π * 0.065)) * 60
    = velocity_m_s * 294.26
```

---

## Performance Metrics (Before Rebuild)

### Test Results (Ki=5.0, with oscillation)
```
Left wheel:  Target 54.65 RPM, Actual 54.94 RPM, Error -0.5% ✅
Right wheel: Target 54.48 RPM, Actual 50.44 RPM, Error 7.4% ⚠️ (oscillating)
```

### Test Results (Ki=2.5, Kd=0.2, dead zone=3 RPM)
```
Not tested yet - system being rebuilt
```

---

## Known Issues

### Issue 1: Right Wheel Oscillation
**Symptom**: Right wheel "twitching" when error < 10 RPM  
**Cause**: Ki=5.0 too aggressive, integral term overshoots  
**Fix**: Reduced Ki to 2.5, increased Kd to 0.2, added 3 RPM dead zone

### Issue 2: Slow Convergence (SOLVED)
**Symptom**: 40-50% error after 10 seconds  
**Cause**: Ki=0.5 too small, takes 94 seconds to accumulate integral  
**Fix**: Increased Ki to 2.5-5.0

### Issue 3: Motor Wiring Swap (CURRENT)
**Symptom**: After swapping IN1 ↔ IN2, only left wheel turns on forward command  
**Cause**: Unknown - requires investigation  
**Action**: Rebuilding PID system from scratch

---

## File Locations

### Source Files
- `src/chassis.cpp` - Chassis control implementation
- `src/pid.cpp` - PID controller implementation
- `src/motor_driver.cpp` - Motor 1 (right wheel) driver
- `src/motor2_driver.cpp` - Motor 2 (left wheel) driver
- `src/encoder.cpp` - Encoder 1 (right wheel)
- `src/encoder2.cpp` - Encoder 2 (left wheel)

### Header Files
- `src/include/chassis.h` - Chassis API
- `src/include/pid.h` - PID API
- `src/include/motor_driver.h` - Motor driver API
- `src/include/encoder.h` - Encoder API

---

## PID Implementation Details

### PID Structure
```cpp
typedef struct {
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain
    float integrator;      // Integral accumulator
    float prev_error;      // Previous error (for derivative)
    float prev_measurement;// Previous measurement (for derivative on measurement)
    float out_min;         // Output minimum limit
    float out_max;         // Output maximum limit
    bool initialized;      // First run flag
} pid_controller_t;
```

### PID Compute Function
```cpp
float pid_compute(pid_controller_t *pid, float setpoint, float measurement, float dt_seconds)
{
    float error = setpoint - measurement;

    // P term
    float p_term = pid->kp * error;

    // I term (with anti-windup)
    pid->integrator += pid->ki * error * dt_seconds;
    float i_max = (pid->out_max - pid->out_min) * 0.8f;  // 80% limit
    float i_term = clampf(pid->integrator, -i_max, i_max);
    pid->integrator = i_term;

    // D term (derivative on measurement to reduce noise)
    float d_term = 0.0f;
    if (pid->initialized) {
        float d_meas = (measurement - pid->prev_measurement) / dt_seconds;
        d_term = -pid->kd * d_meas;
    } else {
        pid->initialized = true;
    }

    // Combine
    float output = p_term + i_term + d_term;
    output = clampf(output, pid->out_min, pid->out_max);

    // Update history
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
}
```

### Dead Zone Implementation
```cpp
static float last_left_pwm = 0.0f;
static float last_right_pwm = 0.0f;

float left_error = fabsf(left_target_rpm) - left_actual_rpm;
float right_error = fabsf(right_target_rpm) - right_actual_rpm;

if (fabsf(left_error) > 3.0f) {
    left_pwm = pid_compute(&g_pid_left, fabsf(left_target_rpm), left_actual_rpm, dt);
    last_left_pwm = left_pwm;
} else {
    left_pwm = last_left_pwm;  // Keep previous PWM
}

if (fabsf(right_error) > 3.0f) {
    right_pwm = pid_compute(&g_pid_right, fabsf(right_target_rpm), right_actual_rpm, dt);
    last_right_pwm = right_pwm;
} else {
    right_pwm = last_right_pwm;  // Keep previous PWM
}
```

---

## Motor Direction Control

### Direction Determination
```cpp
motor_direction_t left_direction = MOTOR_STOP;
motor_direction_t right_direction = MOTOR_STOP;

if (fabsf(left_rpm) > 0.1f) {
    left_direction = (left_rpm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
}
if (fabsf(right_rpm) > 0.1f) {
    right_direction = (right_rpm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
}
```

### Direction Change Handling
```cpp
// Detect direction change
bool left_dir_changed = (left_direction != g_prev_left_direction) &&
                        (g_prev_left_direction != MOTOR_STOP);
bool right_dir_changed = (right_direction != g_prev_right_direction) &&
                         (g_prev_right_direction != MOTOR_STOP);

if (left_dir_changed || right_dir_changed) {
    // Stop motors
    motor_stop();
    motor2_stop();

    // Reset PID
    pid_reset(&g_pid_left);
    pid_reset(&g_pid_right);

    // Set direction change flag
    g_direction_changing = true;

    // Wait 100ms for motors to stop
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set new directions
    motor_set_direction(right_direction);
    motor2_set_direction(left_direction);

    // Clear direction change flag
    g_direction_changing = false;
}
```

---

## Next Steps

1. ✅ Save current configuration to this file
2. ⏳ Investigate motor wiring issue
3. ⏳ Rebuild PID system from scratch with clean architecture
4. ⏳ Test with new motor wiring configuration
5. ⏳ Tune PID parameters for optimal performance

