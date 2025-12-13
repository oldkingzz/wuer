/**
 * @file test_tof_continuous.ino
 * @brief 连续打印两个 ToF 传感器的读数，用于检测是否会自己崩溃
 *
 * ToF 配置:
 *   - SD1 (Channel 1): Front ToF
 *   - SD2 (Channel 2): Left-Front ToF (实际朝右)
 */

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

// TCA9548A I2C 地址
#define TCA9548A_ADDR 0x70

// I2C 引脚
#define I2C_SDA 47
#define I2C_SCL 48

// ToF 传感器对象
Adafruit_VL53L0X lox_front;
Adafruit_VL53L0X lox_right;

bool front_ok = false;
bool right_ok = false;

uint32_t read_count = 0;
uint32_t front_fail_count = 0;
uint32_t right_fail_count = 0;

void tca_select(uint8_t channel) {
  if (channel > 7)
    return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(100);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=====================================");
  Serial.println("ToF Continuous Read Test");
  Serial.println("=====================================");

  // 初始化 I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(100);

  // 检查 TCA9548A
  Wire.beginTransmission(TCA9548A_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: TCA9548A not found!");
    while (1)
      delay(1000);
  }
  Serial.println("TCA9548A: OK");

  // 初始化 Front ToF (SD1 = Channel 1)
  tca_select(1);
  delay(10);
  Serial.print("Init Front ToF (SD1)... ");
  if (lox_front.begin()) {
    Serial.println("OK");
    front_ok = true;
  } else {
    Serial.println("FAILED");
  }

  // 初始化 Right ToF (SD2 = Channel 2)
  tca_select(2);
  delay(10);
  Serial.print("Init Right ToF (SD2)... ");
  if (lox_right.begin()) {
    Serial.println("OK");
    right_ok = true;
  } else {
    Serial.println("FAILED");
  }

  Serial.println();
  Serial.println("Starting continuous read loop...");
  Serial.println("Format: [count] Front=XXXmm Right=XXXmm (F_fail/R_fail)");
  Serial.println();
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  uint16_t front_mm = 0xFFFF;
  uint16_t right_mm = 0xFFFF;
  bool front_valid = false;
  bool right_valid = false;

  read_count++;

  // 读取 Front ToF
  if (front_ok) {
    tca_select(1);
    delay(2);
    lox_front.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8000) {
      front_mm = measure.RangeMilliMeter;
      front_valid = true;
    } else {
      front_fail_count++;
    }
  }

  // 读取 Right ToF
  if (right_ok) {
    tca_select(2);
    delay(2);
    lox_right.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8000) {
      right_mm = measure.RangeMilliMeter;
      right_valid = true;
    } else {
      right_fail_count++;
    }
  }

  // 打印结果
  Serial.printf("[%lu] Front=%s%u%smm  Right=%s%u%smm  (fail: %lu/%lu)\n",
                read_count, front_valid ? "" : "*", front_mm,
                front_valid ? "" : "*", right_valid ? "" : "*", right_mm,
                right_valid ? "" : "*", front_fail_count, right_fail_count);

  // 每 100 次打印统计
  if (read_count % 100 == 0) {
    Serial.println("--- 100 reads ---");
    Serial.printf("Front fail rate: %.1f%%, Right fail rate: %.1f%%\n",
                  (float)front_fail_count / read_count * 100.0f,
                  (float)right_fail_count / read_count * 100.0f);
    Serial.printf("Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    Serial.println();
  }

  delay(50); // ~20Hz
}
