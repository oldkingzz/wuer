/**
 * ============================================================
 * ToF传感器测试代码 / ToF Sensor Test Code
 * ============================================================
 * 
 * 这是一个测试代码，用于测试VL53L0X ToF传感器
 * This is a test code for testing VL53L0X ToF sensors
 * 
 * 测试配置 / Test Configuration:
 * - I2C: SDA=GPIO12, SCL=GPIO13
 * - TCA9548A通道 / TCA9548A Channels:
 *   SD0: Top ToF (浮空/未使用)
 *   SD1: Front ToF (车前)
 *   SD2: Left-Front ToF (车左侧前)
 *   SD3: Left-Rear ToF (车左侧后)
 * 
 * 功能 / Features:
 * - 实时打印所有ToF传感器数据
 * - 无延迟，接收到立即打印
 * - 显示传感器初始化状态
 * - 显示距离、状态码、有效性
 * 
 * ============================================================
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// I2C引脚配置
#define I2C_SDA 12
#define I2C_SCL 13

// 直接测试模式 - 不使用 TCA9548A
// Direct test mode - without TCA9548A
#define USE_TCA9548A 0

// ToF传感器对象 - 只测试一个
Adafruit_VL53L0X lox_test = Adafruit_VL53L0X();

// 初始化标志
bool tof_test_ok = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("  ToF传感器直接测试 / Direct ToF Test");
  Serial.println("  (不使用TCA9548A / Without TCA9548A)");
  Serial.println("========================================");
  Serial.println();
  
  // 初始化I2C
x
  Wire.setClock(100000);  // 100kHz
  delay(100);
  
  Serial.printf("I2C初始化完成 (SDA=GPIO%d, SCL=GPIO%d, 100kHz)\n", I2C_SDA, I2C_SCL);
  Serial.println();
  
  // 扫描I2C设备
  Serial.println("扫描I2C总线...");
  int deviceCount = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  发现设备: 0x%02X\n", addr);
      deviceCount++;
    }
  }
  Serial.printf("找到 %d 个I2C设备\n\n", deviceCount);

  // 初始化ToF传感器（直接连接，地址 0x29）
  Serial.println("初始化ToF传感器 (直接连接到I2C总线)...");
  Serial.println("----------------------------------------");
  Serial.print("尝试初始化 VL53L0X (地址 0x29)... ");

  if (lox_test.begin()) {
    Serial.println("✓ 成功!");
    tof_test_ok = true;
    Serial.println("传感器初始化成功，可以正常读取数据");
  } else {
    Serial.println("✗ 失败!");
    tof_test_ok = false;
    Serial.println("⚠️  传感器初始化失败，请检查:");
    Serial.println("  1. VL53L0X 是否正确连接到 I2C 总线");
    Serial.println("  2. SDA=GPIO12, SCL=GPIO13");
    Serial.println("  3. VCC 和 GND 是否正确连接");
    Serial.println("  4. 传感器是否损坏");
  }

  Serial.println("----------------------------------------");
  Serial.println();

  Serial.println("开始实时读取数据...");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  static unsigned long last_print = 0;
  static unsigned long read_count = 0;
  unsigned long now = millis();

  // 每500ms打印一次
  if (now - last_print < 500) {
    return;
  }
  last_print = now;
  read_count++;

  if (!tof_test_ok) {
    Serial.println("⚠️  传感器未初始化，无法读取数据");
    delay(2000);
    return;
  }

  VL53L0X_RangingMeasurementData_t measure;
  lox_test.rangingTest(&measure, false);

  Serial.println("┌──────────────────────────────────────────────────────────────┐");
  Serial.printf("│ 读取 #%lu | 时间: %lu ms\n", read_count, now);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.print("│ VL53L0X (0x29):   ");

  if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8190) {
    Serial.printf("%4d mm  [Status: %d] ", measure.RangeMilliMeter, measure.RangeStatus);
    // 距离指示条
    int bars = map(measure.RangeMilliMeter, 0, 2000, 20, 0);
    bars = constrain(bars, 0, 20);
    for (int i = 0; i < bars; i++) Serial.print("█");
    Serial.println();
  } else if (measure.RangeMilliMeter >= 8190) {
    Serial.printf("ERROR (65535)  [Status: %d] ✗\n", measure.RangeStatus);
  } else {
    Serial.printf("OUT OF RANGE  [Status: %d] ✗\n", measure.RangeStatus);
  }

  Serial.println("└──────────────────────────────────────────────────────────────┘");
  Serial.println();

  // 提示信息（每10次读取显示一次）
  if (read_count % 10 == 0) {
    Serial.println("💡 提示:");
    Serial.println("   - 距离条: █████ 表示距离（越长越近）");
    Serial.println("   - Status 0 = 正常, 4 = 超出范围");
    Serial.println("   - ERROR (65535) = 传感器读取失败");
    Serial.println("   - 正常范围: 30mm - 2000mm");
    Serial.println("   - 用手靠近传感器测试距离变化");
    Serial.println();
  }
}


