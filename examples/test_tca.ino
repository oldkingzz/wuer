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

// TCA9548A配置
#define TCA9548A_ADDR 0x70
#define USE_TCA9548A 1

// ToF传感器通道
#define TOF_TOP_CHANNEL       0  // SD0 (浮空)
#define TOF_FRONT_CHANNEL     1  // SD1
#define TOF_LEFT_FRONT_CHANNEL 2  // SD2
#define TOF_LEFT_REAR_CHANNEL 3  // SD3

// ToF传感器对象
Adafruit_VL53L0X lox_top = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left_rear = Adafruit_VL53L0X();

// 初始化标志
bool tof_top_ok = false;
bool tof_front_ok = false;
bool tof_left_front_ok = false;
bool tof_left_rear_ok = false;

// TCA9548A通道选择
void tca_select(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("  ToF传感器测试 / ToF Sensor Test");
  Serial.println("========================================");
  Serial.println();
  
  // 初始化I2C
  Wire.begin(I2C_SDA, I2C_SCL);
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

  // 检查TCA9548A是否存在
  Serial.print("检查TCA9548A (地址 0x");
  Serial.print(TCA9548A_ADDR, HEX);
  Serial.println(")...");
  Wire.beginTransmission(TCA9548A_ADDR);
  uint8_t tcaError = Wire.endTransmission();
  if (tcaError == 0) {
    Serial.println("  ✓ TCA9548A 已找到");
  } else {
    Serial.print("  ✗ 未找到TCA9548A, 错误码: ");
    Serial.println(tcaError);
  }
  Serial.println();
  
  // 初始化ToF传感器
  Serial.println("初始化ToF传感器...");
  Serial.println("----------------------------------------");
  
  // SD0: Top ToF (可能浮空)
  tca_select(TOF_TOP_CHANNEL);
  delay(10);
  Serial.print("SD0 (Top ToF, Channel 0)... ");
  if (lox_top.begin()) {
    Serial.println("✓ 成功");
    tof_top_ok = true;
  } else {
    Serial.println("✗ 失败 (可能浮空)");
    tof_top_ok = false;
  }
  
  // SD1: Front ToF
  tca_select(TOF_FRONT_CHANNEL);
  delay(10);
  Serial.print("SD1 (Front ToF, Channel 1)... ");
  if (lox_front.begin()) {
    Serial.println("✓ 成功");
    tof_front_ok = true;
  } else {
    Serial.println("✗ 失败");
    tof_front_ok = false;
  }
  
  // SD2: Left-Front ToF
  tca_select(TOF_LEFT_FRONT_CHANNEL);
  delay(10);
  Serial.print("SD2 (Left-Front ToF, Channel 2)... ");
  if (lox_left_front.begin()) {
    Serial.println("✓ 成功");
    tof_left_front_ok = true;
  } else {
    Serial.println("✗ 失败");
    tof_left_front_ok = false;
  }
  
  // SD3: Left-Rear ToF
  tca_select(TOF_LEFT_REAR_CHANNEL);
  delay(10);
  Serial.print("SD3 (Left-Rear ToF, Channel 3)... ");
  if (lox_left_rear.begin()) {
    Serial.println("✓ 成功");
    tof_left_rear_ok = true;
  } else {
    Serial.println("✗ 失败");
    tof_left_rear_ok = false;
  }

  Serial.println("----------------------------------------");
  Serial.println();

  // 总结初始化结果
  int success_count = tof_top_ok + tof_front_ok + tof_left_front_ok + tof_left_rear_ok;
  Serial.printf("初始化完成: %d/4 个传感器成功\n", success_count);
  Serial.println();

  if (success_count == 0) {
    Serial.println("⚠️  警告: 没有传感器初始化成功!");
    Serial.println("请检查:");
    Serial.println("  1. TCA9548A接线 (SDA=GPIO12, SCL=GPIO13)");
    Serial.println("  2. VL53L0X传感器接线");
    Serial.println("  3. 电源供电是否正常");
    Serial.println();
  }

  Serial.println("开始实时读取数据...");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  static unsigned long last_print = 0;
  static unsigned long read_count = 0;
  unsigned long now = millis();

  // 每200ms打印一次（实时显示，不会太快刷屏）
  if (now - last_print < 200) {
    return;
  }
  last_print = now;
  read_count++;

  VL53L0X_RangingMeasurementData_t measure;

  Serial.println("┌──────────────────────────────────────────────────────────────┐");
  Serial.printf("│ 读取 #%lu | 时间: %lu ms\n", read_count, now);
  Serial.println("├──────────────────────────────────────────────────────────────┤");

  // 读取SD0: Top ToF
  if (tof_top_ok) {
    tca_select(TOF_TOP_CHANNEL);
    lox_top.rangingTest(&measure, false);

    Serial.print("│ SD0 (Top):        ");
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
  } else {
    Serial.println("│ SD0 (Top):        ✗ NOT INITIALIZED");
  }

  // 读取SD1: Front ToF
  if (tof_front_ok) {
    tca_select(TOF_FRONT_CHANNEL);
    lox_front.rangingTest(&measure, false);

    Serial.print("│ SD1 (Front):      ");
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8190) {
      Serial.printf("%4d mm  [Status: %d] ", measure.RangeMilliMeter, measure.RangeStatus);
      int bars = map(measure.RangeMilliMeter, 0, 2000, 20, 0);
      bars = constrain(bars, 0, 20);
      for (int i = 0; i < bars; i++) Serial.print("█");
      Serial.println();
    } else if (measure.RangeMilliMeter >= 8190) {
      Serial.printf("ERROR (65535)  [Status: %d] ✗\n", measure.RangeStatus);
    } else {
      Serial.printf("OUT OF RANGE  [Status: %d] ✗\n", measure.RangeStatus);
    }
  } else {
    Serial.println("│ SD1 (Front):      ✗ NOT INITIALIZED");
  }

  // 读取SD2: Left-Front ToF
  if (tof_left_front_ok) {
    tca_select(TOF_LEFT_FRONT_CHANNEL);
    lox_left_front.rangingTest(&measure, false);

    Serial.print("│ SD2 (Left-Front): ");
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8190) {
      Serial.printf("%4d mm  [Status: %d] ", measure.RangeMilliMeter, measure.RangeStatus);
      int bars = map(measure.RangeMilliMeter, 0, 2000, 20, 0);
      bars = constrain(bars, 0, 20);
      for (int i = 0; i < bars; i++) Serial.print("█");
      Serial.println();
    } else if (measure.RangeMilliMeter >= 8190) {
      Serial.printf("ERROR (65535)  [Status: %d] ✗\n", measure.RangeStatus);
    } else {
      Serial.printf("OUT OF RANGE  [Status: %d] ✗\n", measure.RangeStatus);
    }
  } else {
    Serial.println("│ SD2 (Left-Front): ✗ NOT INITIALIZED");
  }

  // 读取SD3: Left-Rear ToF
  if (tof_left_rear_ok) {
    tca_select(TOF_LEFT_REAR_CHANNEL);
    lox_left_rear.rangingTest(&measure, false);

    Serial.print("│ SD3 (Left-Rear):  ");
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8190) {
      Serial.printf("%4d mm  [Status: %d] ", measure.RangeMilliMeter, measure.RangeStatus);
      int bars = map(measure.RangeMilliMeter, 0, 2000, 20, 0);
      bars = constrain(bars, 0, 20);
      for (int i = 0; i < bars; i++) Serial.print("█");
      Serial.println();
    } else if (measure.RangeMilliMeter >= 8190) {
      Serial.printf("ERROR (65535)  [Status: %d] ✗\n", measure.RangeStatus);
    } else {
      Serial.printf("OUT OF RANGE  [Status: %d] ✗\n", measure.RangeStatus);
    }
  } else {
    Serial.println("│ SD3 (Left-Rear):  ✗ NOT INITIALIZED");
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
    Serial.println();
  }
}

