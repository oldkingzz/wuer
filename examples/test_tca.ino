#include <Wire.h>

#define TCA9548A_ADDR 0x70 // Default I2C address of TCA9548A

void tcaSelect(uint8_t bus) {
  if (bus > 7) return; // Ensure the bus number is valid
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << bus); // Select the specific bus
  uint8_t error = Wire.endTransmission();

  // Debug: verify the write was successful
  if (error != 0) {
    Serial.print("    WARNING: Failed to select channel ");
    Serial.print(bus);
    Serial.print(" (error: ");
    Serial.print(error);
    Serial.println(")");
  }
}

// Read back the current channel selection
uint8_t tcaReadChannel() {
  Wire.requestFrom(TCA9548A_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;  // Error
}

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int deviceCount = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      deviceCount++;
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(" - ");

      // Identify common devices
      if (addr == 0x29) {
        Serial.println("VL53L0X (ToF Sensor)");
      } else if (addr == 0x68) {
        Serial.println("MPU6050 (IMU, AD0=GND)");
      } else if (addr == 0x69) {
        Serial.println("MPU6050 (IMU, AD0=VCC)");
      } else if (addr >= 0x70 && addr <= 0x77) {
        Serial.println("TCA9548A (I2C Multiplexer)");
      } else {
        Serial.println("Unknown device");
      }
    }
  }

  if (deviceCount == 0) {
    Serial.println("  No devices found");
  } else {
    Serial.print("  Total: ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n\n╔════════════════════════════════════════════════════╗");
  Serial.println("║     TCA9548A + Sensor Test / TCA9548A传感器测试    ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");

  // Initialize I2C
  Wire.begin(21, 20);  // SDA=GPIO21, SCL=GPIO20
  Wire.setClock(100000);  // 100kHz
  delay(100);

  Serial.println("I2C initialized (SDA=GPIO21, SCL=GPIO20, 100kHz)\n");

  // Step 1: Scan main I2C bus
  Serial.println("════════════════════════════════════════════════════");
  Serial.println("Step 1: Scanning main I2C bus");
  Serial.println("════════════════════════════════════════════════════");
  scanI2C();

  // Step 2: Check TCA9548A
  Serial.println("════════════════════════════════════════════════════");
  Serial.println("Step 2: Testing TCA9548A");
  Serial.println("════════════════════════════════════════════════════");

  Wire.beginTransmission(TCA9548A_ADDR);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    Serial.print("✓ TCA9548A found at 0x");
    Serial.println(TCA9548A_ADDR, HEX);
    Serial.println();

    // Step 3: Scan each channel
    Serial.println("════════════════════════════════════════════════════");
    Serial.println("Step 3: Scanning TCA9548A channels (0-7)");
    Serial.println("════════════════════════════════════════════════════\n");

    for (uint8_t channel = 0; channel < 8; channel++) {
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.print(": ");

      // Select channel
      tcaSelect(channel);
      delay(50);  // Increased delay

      // Verify channel selection by reading back
      uint8_t readBack = tcaReadChannel();
      uint8_t expected = (1 << channel);

      if (readBack != expected) {
        Serial.println();
        Serial.print("    WARNING: Channel select failed! Expected 0x");
        Serial.print(expected, HEX);
        Serial.print(", got 0x");
        Serial.println(readBack, HEX);
      }

      // Scan for devices on this channel
      int deviceCount = 0;
      for (uint8_t addr = 1; addr < 127; addr++) {
        // Skip TCA9548A address range to avoid confusion
        if (addr >= 0x70 && addr <= 0x77) continue;

        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
          if (deviceCount == 0) {
            Serial.println();  // New line after "Channel X:"
          }
          deviceCount++;
          Serial.print("  ✓ 0x");
          if (addr < 16) Serial.print("0");
          Serial.print(addr, HEX);
          Serial.print(" - ");

          if (addr == 0x29) {
            Serial.println("VL53L0X (ToF Sensor)");
          } else if (addr == 0x68) {
            Serial.println("MPU6050 (IMU, AD0=GND)");
          } else if (addr == 0x69) {
            Serial.println("MPU6050 (IMU, AD0=VCC)");
          } else {
            Serial.println("Unknown device");
          }
        }
      }

      if (deviceCount == 0) {
        Serial.println("(No devices)");
      }

      Serial.println();
    }

    // Close all channels
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(0);
    Wire.endTransmission();

  } else {
    Serial.print("✗ TCA9548A not found at 0x");
    Serial.println(TCA9548A_ADDR, HEX);
  }

  // Step 4: Test if sensors are connected directly to main bus
  Serial.println("\n════════════════════════════════════════════════════");
  Serial.println("Step 4: Checking if sensors are on main bus");
  Serial.println("════════════════════════════════════════════════════");
  Serial.println("(Sensors should NOT be on main bus if using TCA9548A)\n");

  // Close all TCA9548A channels first
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  bool foundOnMainBus = false;

  // Check for VL53L0X
  Wire.beginTransmission(0x29);
  if (Wire.endTransmission() == 0) {
    Serial.println("⚠️  VL53L0X (0x29) found on MAIN bus!");
    Serial.println("    → Should be connected to TCA9548A channel, not main bus");
    foundOnMainBus = true;
  }

  // Check for MPU6050
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    Serial.println("⚠️  MPU6050 (0x68) found on MAIN bus!");
    Serial.println("    → Should be connected to TCA9548A channel, not main bus");
    foundOnMainBus = true;
  }

  Wire.beginTransmission(0x69);
  if (Wire.endTransmission() == 0) {
    Serial.println("⚠️  MPU6050 (0x69) found on MAIN bus!");
    Serial.println("    → Should be connected to TCA9548A channel, not main bus");
    foundOnMainBus = true;
  }

  if (!foundOnMainBus) {
    Serial.println("✓ No sensors on main bus (correct)");
  }

  Serial.println("\n════════════════════════════════════════════════════");
  Serial.println("Test complete! / 测试完成！");
  Serial.println("════════════════════════════════════════════════════\n");

  Serial.println("【诊断建议 / Diagnostic Suggestions】\n");
  Serial.println("如果传感器在主总线上被发现:");
  Serial.println("  → 传感器连接到了ESP32的GPIO 21/20，而不是TCA9548A的通道");
  Serial.println("  → 需要连接到TCA9548A的SD0/SC0 (通道0) 或 SD3/SC3 (通道3)\n");

  Serial.println("如果传感器在通道上找不到:");
  Serial.println("  1. 检查传感器的SDA/SCL是否连接到TCA9548A的SDx/SCx引脚");
  Serial.println("  2. 检查传感器的VCC是否有3.3V");
  Serial.println("  3. 检查传感器的GND是否连接");
  Serial.println("  4. 尝试把传感器直接连接到ESP32 GPIO 21/20测试是否工作\n");

  Serial.println("TCA9548A引脚说明:");
  Serial.println("  主总线: SDA → GPIO 21, SCL → GPIO 20");
  Serial.println("  通道0: SD0, SC0");
  Serial.println("  通道1: SD1, SC1");
  Serial.println("  通道2: SD2, SC2");
  Serial.println("  通道3: SD3, SC3");
  Serial.println("  通道4-7: SD4-7, SC4-7\n");

  Serial.println("Next steps / 下一步:");
  Serial.println("1. Connect VL53L0X to TCA9548A Channel 0 (SD0/SC0)");
  Serial.println("2. Connect MPU6050 to TCA9548A Channel 3 (SD3/SC3)");
  Serial.println("3. Press RESET to test again");
  Serial.println();
}

void loop() {
  // Wait for reset
  delay(10000);
}
