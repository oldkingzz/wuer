// MPU6050 with TCA9548A - Unified Sensors Demo
// Automatically finds MPU6050 on any TCA9548A channel and reads data

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define TCA9548A_ADDR 0x70

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

uint8_t mpu_channel = 255;  // Will be found by scanning
uint8_t mpu_address = 0;    // Will be found by scanning
bool use_adafruit_lib = true;  // Will be set to false if library init fails

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);
}

void setup(void) {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║  MPU6050 + TCA9548A Unified Sensors Test          ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");

  // Initialize I2C
  Wire.begin(21, 20);  // SDA=GPIO21, SCL=GPIO20
  Wire.setClock(100000);  // 100kHz
  delay(100);

  Serial.println("I2C initialized (SDA=GPIO21, SCL=GPIO20, 100kHz)\n");

  // Step 1: Check TCA9548A
  Serial.println("════════════════════════════════════════════════════");
  Serial.println("Step 1: Checking TCA9548A");
  Serial.println("════════════════════════════════════════════════════");
  
  Wire.beginTransmission(TCA9548A_ADDR);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println("✗ TCA9548A not found!");
    Serial.println("STOPPING");
    while(1) delay(1000);
  }
  
  Serial.println("✓ TCA9548A found at 0x70\n");

  // Step 2: Scan all channels for MPU6050
  Serial.println("════════════════════════════════════════════════════");
  Serial.println("Step 2: Scanning all channels for MPU6050");
  Serial.println("════════════════════════════════════════════════════\n");
  
  for (uint8_t ch = 0; ch < 8; ch++) {
    Serial.print("Channel ");
    Serial.print(ch);
    Serial.print(": ");
    
    tcaSelect(ch);
    delay(50);
    
    bool found = false;
    for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
      Wire.beginTransmission(addr);
      error = Wire.endTransmission();
      
      if (error == 0) {
        Serial.print("✓ MPU6050 at 0x");
        Serial.println(addr, HEX);
        mpu_channel = ch;
        mpu_address = addr;
        found = true;
        break;
      }
    }
    
    if (!found) {
      Serial.println("(Empty)");
    }
  }
  
  Serial.println();
  
  if (mpu_channel == 255) {
    Serial.println("✗✗✗ MPU6050 NOT FOUND on any channel! ✗✗✗");
    Serial.println("STOPPING");
    while(1) delay(1000);
  }
  
  Serial.print("✓✓✓ Found MPU6050 on channel ");
  Serial.print(mpu_channel);
  Serial.print(" at address 0x");
  Serial.print(mpu_address, HEX);
  Serial.println(" ✓✓✓\n");

  // Step 3: Wake up MPU6050 manually
  Serial.println("════════════════════════════════════════════════════");
  Serial.println("Step 3: Waking up MPU6050");
  Serial.println("════════════════════════════════════════════════════");

  tcaSelect(mpu_channel);
  delay(50);

  Serial.print("Writing to PWR_MGMT_1 (0x6B) to wake up... ");
  Wire.beginTransmission(mpu_address);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Clear SLEEP bit
  error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("FAILED! Error: ");
    Serial.println(error);
    Serial.println("STOPPING");
    while(1) delay(1000);
  }
  Serial.println("SUCCESS!");
  delay(100);
  Serial.println();

  // Step 4: Verify WHO_AM_I
  Serial.println("════════════════════════════════════════════════════");
  Serial.println("Step 4: Reading WHO_AM_I register");
  Serial.println("════════════════════════════════════════════════════");

  tcaSelect(mpu_channel);
  delay(50);

  Wire.beginTransmission(mpu_address);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, (uint8_t)1);

  if (Wire.available()) {
    uint8_t whoami = Wire.read();
    Serial.print("WHO_AM_I = 0x");
    Serial.print(whoami, HEX);
    if (whoami == 0x68 || whoami == 0x70) {
      Serial.println(" ✓ (Valid MPU6050)");
    } else {
      Serial.println(" ✗ (Unexpected value!)");
    }
  } else {
    Serial.println("✗ Failed to read WHO_AM_I");
  }
  Serial.println();

  // Step 5: Initialize MPU6050 with Adafruit library
  Serial.println("════════════════════════════════════════════════════");
  Serial.println("Step 5: Initializing Adafruit MPU6050 library");
  Serial.println("════════════════════════════════════════════════════");

  // Keep selecting channel before EVERY library call
  tcaSelect(mpu_channel);
  delay(100);

  Serial.print("Calling mpu.begin(0x");
  Serial.print(mpu_address, HEX);
  Serial.print(", &Wire)... ");

  if (!mpu.begin(mpu_address, &Wire)) {
    Serial.println("FAILED!");
    Serial.println("\n⚠️  Adafruit library initialization failed.");
    Serial.println("    This is a known issue with TCA9548A.");
    Serial.println("    Will read data manually instead...\n");

    use_adafruit_lib = false;
  } else {
    Serial.println("SUCCESS!\n");
    use_adafruit_lib = true;
  }

  // Step 6: Get sensor objects (only if Adafruit library works)
  if (use_adafruit_lib) {
    Serial.println("════════════════════════════════════════════════════");
    Serial.println("Step 6: Getting unified sensor objects");
    Serial.println("════════════════════════════════════════════════════");

    tcaSelect(mpu_channel);
    delay(50);

    mpu_temp = mpu.getTemperatureSensor();
    mpu_accel = mpu.getAccelerometerSensor();
    mpu_gyro = mpu.getGyroSensor();

    Serial.println("✓ Temperature sensor");
    Serial.println("✓ Accelerometer sensor");
    Serial.println("✓ Gyroscope sensor\n");

    // Print sensor details
    Serial.println("Temperature Sensor Details:");
    mpu_temp->printSensorDetails();
    Serial.println();

    Serial.println("Accelerometer Sensor Details:");
    mpu_accel->printSensorDetails();
    Serial.println();

    Serial.println("Gyroscope Sensor Details:");
    mpu_gyro->printSensorDetails();
    Serial.println();
  } else {
    Serial.println("════════════════════════════════════════════════════");
    Serial.println("Step 6: Configuring MPU6050 manually");
    Serial.println("════════════════════════════════════════════════════");

    tcaSelect(mpu_channel);
    delay(50);

    // Set accelerometer range to ±8g (register 0x1C, value 0x10)
    Serial.print("Setting accel range to ±8g... ");
    Wire.beginTransmission(mpu_address);
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(0x10);  // ±8g (bits 4:3 = 10)
    Wire.endTransmission();
    Serial.println("OK");
    delay(10);

    tcaSelect(mpu_channel);
    delay(10);

    // Set gyro range to ±500°/s (register 0x1B, value 0x08)
    Serial.print("Setting gyro range to ±500°/s... ");
    Wire.beginTransmission(mpu_address);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0x08);  // ±500°/s (bits 4:3 = 01)
    Wire.endTransmission();
    Serial.println("OK");
    delay(10);

    Serial.println();
  }

  Serial.println("════════════════════════════════════════════════════");
  Serial.println("Starting continuous data reading...");
  Serial.println("════════════════════════════════════════════════════\n");
  delay(1000);
}

// Helper function to read 16-bit signed value from two registers
int16_t readRegister16(uint8_t reg) {
  tcaSelect(mpu_channel);
  delay(5);

  Wire.beginTransmission(mpu_address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, (uint8_t)2);

  if (Wire.available() >= 2) {
    int16_t value = Wire.read() << 8 | Wire.read();
    return value;
  }
  return 0;
}

void loop() {
  if (use_adafruit_lib) {
    // Use Adafruit library
    tcaSelect(mpu_channel);
    delay(50);

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mpu_temp->getEvent(&temp);
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

    // Display temperature
    Serial.print("\t\tTemperature ");
    Serial.print(temp.temperature);
    Serial.println(" deg C");

    // Display acceleration (m/s^2)
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");

    // Display gyroscope (rad/s)
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();

  } else {
    // Manual reading
    tcaSelect(mpu_channel);
    delay(50);

    // Read temperature (register 0x41-0x42)
    int16_t temp_raw = readRegister16(0x41);
    float temperature = (temp_raw / 340.0) + 36.53;

    // Read accelerometer (registers 0x3B-0x40)
    int16_t accel_x_raw = readRegister16(0x3B);
    int16_t accel_y_raw = readRegister16(0x3D);
    int16_t accel_z_raw = readRegister16(0x3F);

    // Convert to m/s² (±8g range: 4096 LSB/g)
    float accel_x = (accel_x_raw / 4096.0) * 9.81;
    float accel_y = (accel_y_raw / 4096.0) * 9.81;
    float accel_z = (accel_z_raw / 4096.0) * 9.81;

    // Read gyroscope (registers 0x43-0x48)
    int16_t gyro_x_raw = readRegister16(0x43);
    int16_t gyro_y_raw = readRegister16(0x45);
    int16_t gyro_z_raw = readRegister16(0x47);

    // Convert to rad/s (±500°/s range: 65.5 LSB/(°/s))
    float gyro_x = (gyro_x_raw / 65.5) * (3.14159 / 180.0);
    float gyro_y = (gyro_y_raw / 65.5) * (3.14159 / 180.0);
    float gyro_z = (gyro_z_raw / 65.5) * (3.14159 / 180.0);

    // Display temperature
    Serial.print("\t\tTemperature ");
    Serial.print(temperature);
    Serial.println(" deg C");

    // Display acceleration (m/s^2)
    Serial.print("\t\tAccel X: ");
    Serial.print(accel_x);
    Serial.print(" \tY: ");
    Serial.print(accel_y);
    Serial.print(" \tZ: ");
    Serial.print(accel_z);
    Serial.println(" m/s^2 ");

    // Display gyroscope (rad/s)
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro_x);
    Serial.print(" \tY: ");
    Serial.print(gyro_y);
    Serial.print(" \tZ: ");
    Serial.print(gyro_z);
    Serial.println(" radians/s ");
    Serial.println();
  }

  delay(500);
}

