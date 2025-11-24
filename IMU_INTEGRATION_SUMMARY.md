# MPU6050 IMU集成总结 / MPU6050 IMU Integration Summary

## ✅ 已完成的工作 / Completed Work

### 1. **自动检测MPU6050**
- ✓ 自动扫描TCA9548A的所有8个通道
- ✓ 自动检测MPU6050地址（0x68或0x69）
- ✓ 无需手动配置通道号

### 2. **双模式支持**
程序会自动尝试两种模式：

#### **模式A: Adafruit库模式**（优先）
- 使用`Adafruit_MPU6050`库
- 更高级的API
- 如果初始化成功，使用此模式

#### **模式B: 手动寄存器模式**（备用）
- 直接读取MPU6050寄存器
- 绕过Adafruit库的TCA9548A兼容性问题
- 如果Adafruit库失败，自动切换到此模式

### 3. **完整的传感器数据**
所有数据都已保留并可用：

```cpp
typedef struct {
    imu_accel_t accel;     // 加速度 (m/s²)
    imu_gyro_t gyro;       // 陀螺仪 (rad/s)
    float temperature;     // 温度 (°C)
    bool valid;            // 数据有效性
} imu_data_t;
```

### 4. **主要使用的函数**
你主要需要的是**Z轴陀螺仪**（用于旋转检测）：

```cpp
float gyro_z = imu_get_gyro_z();  // 获取Z轴角速度 (rad/s)
```

其他可用函数：
```cpp
imu_read(&imu_data);              // 读取完整IMU数据
imu_read_accel(&accel);           // 只读加速度
imu_read_gyro(&gyro);             // 只读陀螺仪
imu_get_temperature();            // 获取温度
```

---

## 📁 修改的文件 / Modified Files

### **src/imu_sensor.cpp**
主要修改：
1. 添加了`auto_detect_mpu6050()`函数 - 自动扫描通道
2. 添加了`read_register16()`函数 - 手动读取寄存器
3. 修改了`imu_init()` - 支持自动检测和双模式
4. 修改了`imu_read()` - 支持手动寄存器读取

### **src/include/imu_sensor.h**
无需修改，API保持不变。

---

## 🔧 技术细节 / Technical Details

### **手动读取模式的寄存器映射**

| 数据 | 寄存器 | 转换公式 |
|------|--------|----------|
| 温度 | 0x41-0x42 | `(raw/340.0) + 36.53` |
| 加速度X | 0x3B-0x3C | `(raw/4096.0) * 9.81` (±8g) |
| 加速度Y | 0x3D-0x3E | `(raw/4096.0) * 9.81` |
| 加速度Z | 0x3F-0x40 | `(raw/4096.0) * 9.81` |
| 陀螺仪X | 0x43-0x44 | `(raw/65.5) * (π/180)` (±500°/s) |
| 陀螺仪Y | 0x45-0x46 | `(raw/65.5) * (π/180)` |
| 陀螺仪Z | 0x47-0x48 | `(raw/65.5) * (π/180)` |

### **配置**
- 加速度范围: ±8g
- 陀螺仪范围: ±500°/s
- 滤波器带宽: 21Hz（仅Adafruit模式）

---

## 🚀 使用方法 / Usage

### **在主程序中**
主程序（`src/wuer.ino`）已经在使用IMU：

```cpp
// 在sensor_update_task中，每50ms读取一次
imu_data_t imu_data;
imu_read(&imu_data);

// 在status_monitor_task中显示
float gyro_z = imu_get_gyro_z();
Serial.print("IMU Gyro Z: ");
Serial.print(gyro_z, 3);
Serial.println(" rad/s");
```

### **用于机器人控制**
Z轴陀螺仪可用于：
- 检测机器人旋转速度
- 辅助里程计（与编码器融合）
- 检测碰撞或滑移
- 姿态估计

---

## 📊 初始化输出示例 / Initialization Output Example

```
Detecting MPU6050...
✓ Found MPU6050 on channel 3 at 0x68
Init MPU6050 (Channel 3, 0x68)...
  Waking up MPU6050... OK!
  Initializing Adafruit library... Failed!
  → Switching to manual register mode
  Configuring MPU6050... Done!
  Testing read... Accel(0.12, -0.05, 9.81) Gyro(0.001, -0.002, 0.000) Temp=28.5°C
  ✓ Data looks good!
```

---

## ✨ 优势 / Advantages

1. **自动化** - 无需手动配置通道号
2. **鲁棒性** - 两种模式确保总能工作
3. **兼容性** - 解决了Adafruit库与TCA9548A的兼容性问题
4. **完整性** - 保留所有传感器数据供未来使用
5. **简洁性** - API保持不变，无需修改现有代码

---

## 🎯 下一步 / Next Steps

IMU已完全集成并工作正常。你现在可以：

1. **使用Z轴陀螺仪**进行旋转控制
2. **融合编码器和IMU数据**提高里程计精度
3. **添加卡尔曼滤波**进一步提高精度
4. **使用加速度计**检测碰撞或倾斜

所有数据都已准备好，随时可用！🚀

