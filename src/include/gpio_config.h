/**
 * @file gpio_config.h
 * @brief GPIO引脚配置定义 / GPIO Pin Configuration Definitions
 *
 * 本文件定义了ESP32项目中所有使用的GPIO引脚
 * This file defines all GPIO pins used in the ESP32 project
 */

#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include "driver/gpio.h"
#include "hal/adc_types.h"

/* ========== 电机驱动器引脚 / Motor Driver Pins (L298N) ========== */

/**
 * L298N IN1 引脚 - 控制电机方向位1
 * L298N IN1 Pin - Motor direction control bit 1
 */
#define MOTOR_IN1_GPIO GPIO_NUM_6

/**
 * L298N IN2 引脚 - 控制电机方向位2
 * L298N IN2 Pin - Motor direction control bit 2
 */
#define MOTOR_IN2_GPIO GPIO_NUM_7

/**
 * L298N ENA (PWM) 引脚 - 控制电机速度
 * L298N ENA (PWM) Pin - Motor speed control
 * 注意：避免使用GPIO 0, 3, 9, 45, 46 (Strapping Pins)
 * Note: Avoid GPIO 0, 3, 9, 45, 46 (Strapping Pins)
 */
#define MOTOR_PWM_GPIO GPIO_NUM_15

/* ========== 电机2驱动器引脚 / Motor 2 Driver Pins (L298N) ========== */

/**
 * L298N IN3 引脚 - 控制电机2方向位1
 * L298N IN3 Pin - Motor 2 direction control bit 1
 */
#define MOTOR2_IN1_GPIO GPIO_NUM_38

/**
 * L298N IN4 引脚 - 控制电机2方向位2
 * L298N IN4 Pin - Motor 2 direction control bit 2
 */
#define MOTOR2_IN2_GPIO GPIO_NUM_37

/**
 * L298N ENB (PWM) 引脚 - 控制电机2速度
 * L298N ENB (PWM) Pin - Motor 2 speed control
 */
#define MOTOR2_PWM_GPIO GPIO_NUM_39

/* ========== 编码器引脚 / Encoder Pins ========== */

/**
 * 编码器A相信号引脚 (黄色线)
 * Encoder Channel A Signal Pin (Yellow wire)
 */
#define ENCODER_A_GPIO GPIO_NUM_4

/**
 * 编码器B相信号引脚 (绿色线)
 * Encoder Channel B Signal Pin (Green wire)
 */
#define ENCODER_B_GPIO GPIO_NUM_5

/* ========== 编码器2引脚 / Encoder 2 Pins ========== */

/**
 * 编码器2 A相信号引脚 (黄色线)
 * Encoder 2 Channel A Signal Pin (Yellow wire)
 */
#define ENCODER2_A_GPIO GPIO_NUM_40

/**
 * 编码器2 B相信号引脚 (绿色线)
 * Encoder 2 Channel B Signal Pin (Green wire)
 */
#define ENCODER2_B_GPIO GPIO_NUM_41

/* ========== I2C总线引脚 / I2C Bus Pins ========== */

/**
 * I2C SDA引脚 - 数据线
 * I2C SDA Pin - Data Line
 *
 * 硬件实际接线: SDA = GPIO12
 */
#define I2C_SDA_GPIO GPIO_NUM_10

/**
 * I2C SCL引脚 - 时钟线
 * I2C SCL Pin - Clock Line
 *
 * 硬件实际接线: SCL = GPIO9
 */
#define I2C_SCL_GPIO GPIO_NUM_9

/**
 * I2C总线频率
 * I2C Bus Frequency
 */
#define I2C_FREQ_HZ 40000 // 40kHz (Lower speed for Tophat/Slave compatibility)

/**
 * TCA9548A I2C多路复用器配置
 * TCA9548A I2C Multiplexer Configuration
 */
#define USE_TCA9548A 1     // 0=不使用TCA9548A（直连），1=使用TCA9548A
#define TCA9548A_ADDR 0x70 // TCA9548A地址

/* ========== 传感器通道定义 / Sensor Channel Definitions ========== */

/**
 * VL53L0X ToF传感器通道分配 (通过TCA9548A)
 * VL53L0X ToF Sensor Channel Assignment (via TCA9548A)
 *
 * 新的通道分配 / New channel assignment:
 * SD0: 顶部ToF (保留) / Top ToF (reserved)
 * SD1: 车前ToF / Front ToF
 * SD2: 车左侧前面ToF / Left-Front ToF
 * SD3: 车左侧后面ToF / Left-Rear ToF
 * SD4: MPU6050 IMU
 */
#define TOF_TOP_CHANNEL 0        // 顶部ToF传感器 / Top ToF (SD0)
#define TOF_FRONT_CHANNEL 1      // 车前ToF传感器 / Front ToF (SD1)
#define TOF_LEFT_FRONT_CHANNEL 2 // 车左侧前面ToF / Left-Front ToF (SD2)
#define TOF_LEFT_REAR_CHANNEL 3  // 车左侧后面ToF / Left-Rear ToF (SD3)

// 为了兼容旧代码，保留旧的宏定义
#define TOF_LEFT_CHANNEL TOF_LEFT_FRONT_CHANNEL // 兼容旧代码
#define TOF_RIGHT_CHANNEL TOF_FRONT_CHANNEL     // 兼容旧代码

/**
 * MPU6050 IMU传感器通道 (通过TCA9548A)
 * MPU6050 IMU Sensor Channel (via TCA9548A)
 */
#define IMU_CHANNEL 4 // IMU传感器 / IMU sensor (SD4)

/* ========== Vive定位传感器引脚 / Vive Positioning Sensor Pins ========== */

/**
 * Vive传感器1信号引脚
 * Vive Sensor 1 Signal Pin
 */
#define VIVE1_SIGNAL_GPIO GPIO_NUM_16

/**
 * Vive传感器2信号引脚
 * Vive Sensor 2 Signal Pin
 */
#define VIVE2_SIGNAL_GPIO GPIO_NUM_42

/* ========== 用户输入引脚 / User Input Pins ========== */

/**
 * 电位器模拟输入引脚 - 用于控制PWM占空比
 * Potentiometer Analog Input Pin - Controls PWM duty cycle
 * ESP32-S3: 使用 GPIO 16 (ADC2_CH5)
 * ADC2可用通道: CH0-CH9 对应 GPIO 11-20
 */
#define POT_ADC_GPIO GPIO_NUM_16
#define POT_ADC_CHANNEL ADC_CHANNEL_5 // GPIO 16 = ADC2_CH5

/**
 * 按钮开关引脚 - 控制电机启动/停止
 * Push Button Pin - Controls motor START/STOP
 * ESP32-S3: 使用 GPIO 7
 * 使用内部上拉电阻，按下时为低电平
 * Uses internal pull-up, active LOW when pressed
 */
#define BUTTON_GPIO GPIO_NUM_42

/* ========== 编码器参数 / Encoder Parameters ========== */

/**
 * 编码器每转脉冲数 (CPR - Counts Per Revolution)
 * Encoder counts per motor-shaft revolution
 *
 * 数据手册已说明: "64 counts per revolution of the motor shaft
 * when counting both edges of both channels"，也就是已经是4倍频后的计数。
 * Datasheet: 64 counts/rev when counting both edges of both channels
 * (this already includes 4x quadrature decoding).
 */
#define ENCODER_BASE_CPR 64 // 64 counts / motor rev at 4x

/**
 * 为了兼容旧命名，这里仍然保留 ENCODER_CPR_4X，但数值与 ENCODER_BASE_CPR 相同。
 * For backward compatibility, ENCODER_CPR_4X is kept but equals
 * ENCODER_BASE_CPR.
 */
#define ENCODER_CPR_4X (ENCODER_BASE_CPR) // 64 effective counts

/**
 * 电机减速比
 * Motor Gear Reduction Ratio
 * 新电机: 50:1
 * New motor: 50:1
 */
#define MOTOR_GEAR_RATIO 50

/**
 * 轮子转一圈的总计数 (编码器有效CPR × 减速比)
 * Total counts per wheel revolution (effective encoder CPR × gear ratio)
 *
 * 数据手册: 64 counts/rev (motor shaft, 4x) × 50:1 = 3200 counts/rev (output
 * shaft) Datasheet: 64 × 50 = 3200 counts per gearbox output revolution
 */
#define ENCODER_CPR (ENCODER_CPR_4X * MOTOR_GEAR_RATIO) // 64 × 50 = 3200

/* ========== PWM参数 / PWM Parameters ========== */

/**
 * PWM频率 (Hz)
 * PWM Frequency in Hz
 * 典型值: 1000-20000 Hz，这里使用5kHz
 * Typical: 1000-20000 Hz, using 5kHz here
 */
#define MOTOR_PWM_FREQ_HZ 5000

/**
 * PWM占空比分辨率 (位)
 * PWM Duty Cycle Resolution (bits)
 * 10位 = 0-1023 范围
 * 10-bit = 0-1023 range
 */
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT

/**
 * PWM最大占空比值
 * Maximum PWM duty cycle value
 */
#define MOTOR_PWM_MAX_DUTY ((1 << 10) - 1) // 1023

/* ========== ADC参数 / ADC Parameters ========== */

/**
 * ADC最大值
 * ADC Maximum Value
 * 12位ADC: 0-4095 范围
 * 12-bit ADC: 0-4095 range
 */
#define ADC_MAX_VALUE 4095

/* ========== 按钮去抖参数 / Button Debounce Parameters ========== */

/**
 * 按钮去抖延迟 (毫秒)
 * Button debounce delay in milliseconds
 */
#define BUTTON_DEBOUNCE_MS 50

/* ========== 电机参数 / Motor Parameters ========== */

/**
 * 电机额定电压
 * Motor Rated Voltage
 */
#define MOTOR_RATED_VOLTAGE 12.0f

/**
 * 电机空载转速 (RPM)
 * Motor No-Load Speed in RPM
 * 新电机参数（需要根据实际电机更新）
 * New motor parameters (update based on actual motor specs)
 */
#define MOTOR_NO_LOAD_RPM 130

/**
 * 电机额定扭矩 (kg.cm)
 * Motor Rated Torque in kg.cm
 * 新电机参数（需要根据实际电机更新）
 * New motor parameters (update based on actual motor specs)
 */
#define MOTOR_RATED_TORQUE 1.2f

#endif // GPIO_CONFIG_H
