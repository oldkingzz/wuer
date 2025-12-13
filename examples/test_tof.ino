#include "Adafruit_VL53L0X.h"
#include <Wire.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  // ==========================================
  // 关键修正: 适配你的硬件配置
  // Critical Fix: Adapt to your hardware
  // ==========================================

  // 1. 设置正确的 I2C 引脚 (SDA=10, SCL=9)
  // 1. Set correct I2C pins (SDA=10, SCL=9)
  Wire.begin(10, 9);

  // 2. 配置 TCA9548A 多路复用器 (地址 0x70)
  // 2. Configure TCA9548A Multiplexer (Address 0x70)
  // 必须先选通一个通道，否则 ToF 传感器不可见
  // Must select a channel first, otherwise ToF is invisible
  Wire.beginTransmission(0x70);
  Wire.write(1 << 1); // 选择通道 1 (SD1 / Front ToF)
  Wire.endTransmission();
  delay(10);

  // ==========================================

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure,
                  false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) { // phase failures have incorrect data
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

  delay(100);
}
