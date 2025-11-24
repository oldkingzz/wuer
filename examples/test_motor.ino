/**
 * @file test_motor_encoder.ino
 * @brief 电机和编码器测试程序 / Motor and Encoder Test Program
 *
 * 测试内容 / Test Items:
 * 1. 电机驱动器PWM输出
 * 2. 电机方向控制
 * 3. 编码器脉冲计数
 * 4. 编码器转速测量
 *
 * 硬件连接 / Hardware Connections:
 * Motor 1 (Right):
 *   - IN1: GPIO 7
 *   - IN2: GPIO 6
 *   - ENA (PWM): GPIO 15
 *   - Encoder A: GPIO 4
 *   - Encoder B: GPIO 5
 *
 * Motor 2 (Left):
 *   - IN3: GPIO 37
 *   - IN4: GPIO 38
 *   - ENB (PWM): GPIO 39
 *   - Encoder A: GPIO 40
 *   - Encoder B: GPIO 41
 */

#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

// Motor 1 pins
#define MOTOR1_IN1    7
#define MOTOR1_IN2    6
#define MOTOR1_PWM    15
#define ENCODER1_A    4
#define ENCODER1_B    5

// Motor 2 pins
#define MOTOR2_IN1    37
#define MOTOR2_IN2    38
#define MOTOR2_PWM    39
#define ENCODER2_A    40
#define ENCODER2_B    41

// PWM configuration
#define PWM_FREQ      20000
#define PWM_RESOLUTION 10
#define PWM_MAX       1023

// Encoder counters
volatile int32_t encoder1_count = 0;
volatile int32_t encoder2_count = 0;

// Encoder ISR for Motor 1
void IRAM_ATTR encoder1_isr() {
  static uint8_t last_state = 0;
  uint8_t a = digitalRead(ENCODER1_A);
  uint8_t b = digitalRead(ENCODER1_B);
  uint8_t state = (a << 1) | b;
  uint8_t combined = (last_state << 2) | state;

  // Forward transitions
  if (combined == 0b0001 || combined == 0b0111 || combined == 0b1110 || combined == 0b1000) {
    encoder1_count++;
  }
  // Backward transitions
  else if (combined == 0b0010 || combined == 0b1011 || combined == 0b1101 || combined == 0b0100) {
    encoder1_count--;
  }

  last_state = state;
}

// Encoder ISR for Motor 2
void IRAM_ATTR encoder2_isr() {
  static uint8_t last_state = 0;
  uint8_t a = digitalRead(ENCODER2_A);
  uint8_t b = digitalRead(ENCODER2_B);
  uint8_t state = (a << 1) | b;
  uint8_t combined = (last_state << 2) | state;

  // Forward transitions
  if (combined == 0b0001 || combined == 0b0111 || combined == 0b1110 || combined == 0b1000) {
    encoder2_count++;
  }
  // Backward transitions
  else if (combined == 0b0010 || combined == 0b1011 || combined == 0b1101 || combined == 0b0100) {
    encoder2_count--;
  }

  last_state = state;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println();
  Serial.println("====================================================");
  Serial.println("     Motor and Encoder Test Program");
  Serial.println("====================================================");
  Serial.println();

  // Configure motor direction pins
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  // Stop motors initially
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);

  Serial.println("Motor direction pins configured");

  // Configure PWM for Motor 1 using LEDC driver API
  ledc_timer_config_t ledc_timer1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer1);

  ledc_channel_config_t ledc_channel1 = {
    .gpio_num = MOTOR1_PWM,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel1);

  Serial.print("Motor 1 PWM configured: GPIO ");
  Serial.print(MOTOR1_PWM);
  Serial.print(" @ ");
  Serial.print(PWM_FREQ);
  Serial.println(" Hz");

  // Configure PWM for Motor 2 using LEDC driver API
  ledc_timer_config_t ledc_timer2 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_1,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer2);

  ledc_channel_config_t ledc_channel2 = {
    .gpio_num = MOTOR2_PWM,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel2);

  Serial.print("Motor 2 PWM configured: GPIO ");
  Serial.print(MOTOR2_PWM);
  Serial.print(" @ ");
  Serial.print(PWM_FREQ);
  Serial.println(" Hz");

  // Configure encoder pins
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B), encoder1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B), encoder2_isr, CHANGE);

  Serial.println("Encoder interrupts configured");
  Serial.println();
  Serial.println("====================================================");
  Serial.println("Test Sequence:");
  Serial.println("  1. Motor 1 Forward @ 30% speed (5 sec)");
  Serial.println("  2. Motor 1 Stop (2 sec)");
  Serial.println("  3. Motor 2 Forward @ 30% speed (5 sec)");
  Serial.println("  4. Motor 2 Stop (2 sec)");
  Serial.println("  5. Both motors Forward @ 50% speed (5 sec)");
  Serial.println("  6. Stop all");
  Serial.println("====================================================");
  Serial.println();

  delay(2000);
}

void loop() {
  // Test 1: Motor 1 Forward
  Serial.println("----------------------------------------------------");
  Serial.println("TEST 1: Motor 1 (Right) Forward @ 30%");
  Serial.println("----------------------------------------------------");

  encoder1_count = 0;
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 307);  // 30% of 1023
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  for (int i = 0; i < 5; i++) {
    delay(1000);
    Serial.print("  Time: ");
    Serial.print(i + 1);
    Serial.print("s  |  Encoder 1: ");
    Serial.print(encoder1_count);
    Serial.print("  |  PWM: ");
    Serial.println(ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
  }

  // Stop Motor 1
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  Serial.println("Motor 1 stopped");
  Serial.print("Total encoder counts: ");
  Serial.println(encoder1_count);
  Serial.println();
  delay(2000);

  // Test 2: Motor 2 Forward
  Serial.println("----------------------------------------------------");
  Serial.println("TEST 2: Motor 2 (Left) Forward @ 30%");
  Serial.println("----------------------------------------------------");

  encoder2_count = 0;
  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 307);  // 30% of 1023
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

  for (int i = 0; i < 5; i++) {
    delay(1000);
    Serial.print("  Time: ");
    Serial.print(i + 1);
    Serial.print("s  |  Encoder 2: ");
    Serial.print(encoder2_count);
    Serial.print("  |  PWM: ");
    Serial.println(ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
  }

  // Stop Motor 2
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

  Serial.println("Motor 2 stopped");
  Serial.print("Total encoder counts: ");
  Serial.println(encoder2_count);
  Serial.println();
  delay(2000);

  // Test 3: Both motors Forward
  Serial.println("----------------------------------------------------");
  Serial.println("TEST 3: Both Motors Forward @ 50%");
  Serial.println("----------------------------------------------------");

  encoder1_count = 0;
  encoder2_count = 0;

  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);  // 50% of 1023
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 512);  // 50% of 1023
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

  for (int i = 0; i < 5; i++) {
    delay(1000);
    Serial.print("  Time: ");
    Serial.print(i + 1);
    Serial.print("s  |  Enc1: ");
    Serial.print(encoder1_count);
    Serial.print("  |  Enc2: ");
    Serial.println(encoder2_count);
  }

  // Stop all motors
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

  Serial.println("All motors stopped");
  Serial.print("Motor 1 total counts: ");
  Serial.println(encoder1_count);
  Serial.print("Motor 2 total counts: ");
  Serial.println(encoder2_count);
  Serial.println();

  Serial.println("====================================================");
  Serial.println("Test complete! Waiting 10 seconds before restart...");
  Serial.println("====================================================");
  Serial.println();

  delay(10000);
}

