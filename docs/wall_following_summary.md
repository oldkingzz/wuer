# 寻墙算法总结 (Wall Following System)

**文件位置**：`src/wall_following.cpp`, `src/include/wall_following.h`
**最后更新**：2025-12-09
**状态**：✅ 已集成Chassis V2，调试输出已增强

---

## 📋 **系统概述**

### **功能**
沿ROBA 2025地图边缘逆时针行驶一圈，用于探索和定位。

### **起始位置**
- **位置**：右上角（北侧Nexus右侧）
- **朝向**：朝北（90°）
- **假设**：左侧有墙，前方有墙

### **传感器配置**
- **ToF Front**：前方距离检测（SD1）
- **ToF Left-Front**：左侧距离检测（SD2，用于循墙）
- **IMU**：陀螺仪Z轴（朝向估计）
- **Encoder**：左右轮编码器（里程计）

---

## 🔄 **状态机**

### **状态定义**
```cpp
typedef enum {
    WF_STATE_IDLE,          // 空闲
    WF_STATE_INIT_POS,      // 计算初始位置
    WF_STATE_FOLLOW_WALL,   // 循墙前进
    WF_STATE_STOPPED        // 停止
} wall_follow_state_t;
```

### **状态转换**
```
IDLE
  ↓ wall_following_start()
INIT_POS (计算初始位置)
  ↓ calc_init_pos() 成功
FOLLOW_WALL (PID循墙)
  ↓ wall_following_stop()
STOPPED
  ↓ 任务删除
IDLE
```

---

## ⚙️ **核心参数**

### **地图尺寸**
```cpp
#define MAP_WIDTH   60.0f   // 英寸
#define MAP_HEIGHT  144.0f  // 英寸
```

### **车辆参数**
```cpp
#define ROBOT_RADIUS              5.0f     // 碰撞半径（英寸）
#define CHASSIS_WHEEL_DIAMETER_M  0.065f   // 轮径 (m)
#define CHASSIS_WHEEL_BASE_M      0.15f    // 轮距 (m)
```

### **控制参数**
```cpp
#define FORWARD_SPEED       0.30f   // 前进速度 (m/s)
#define TURN_SPEED          1.0f    // 转向速度 (rad/s)
#define WALL_TARGET_DIST    80.0f   // 目标墙距 (mm)
#define PID_KP              0.01f   // PID比例系数
```

---

## 🧮 **核心算法**

### **1. 初始位置计算** (`calc_init_pos`)

**输入**：ToF Front, ToF Left
**输出**：初始位置 (x, y, heading)

**算法**：
```cpp
// 采样10次取平均
for (int i = 0; i < 10; i++) {
    left = tof_get_cached_left_front_distance();
    front = tof_get_cached_front_distance();
    if (valid) { left_sum += left; front_sum += front; }
}

// 计算位置（假设在右上角）
x = MM_TO_INCHES(avg_left) + ROBOT_RADIUS;
y = MAP_HEIGHT - MM_TO_INCHES(avg_front) - ROBOT_RADIUS;
heading = 90.0°;  // 朝北
```

**有效性检查**：
- 至少3个有效样本
- ToF读数：0 < distance < 1000mm

---

### **2. PID循墙控制** (`wall_follow_pid`)

**输入**：目标墙距 (80mm), 当前墙距
**输出**：角速度指令 (rad/s)

**算法**：
```cpp
error = target - current;
angular_velocity = Kp * error;
```

**控制逻辑**：
```cpp
if (tof_left > 0 && tof_left < 500) {
    // 有效墙距，PID控制
    angular = wall_follow_pid(80.0f, tof_left);
    chassis_v2_set_velocity(0.30f, angular);
} else {
    // 墙壁丢失，直行
    chassis_v2_set_velocity(0.30f, 0.0f);
}
```

---

### **3. 里程计更新** (`update_odometry`)

**输入**：编码器RPM, IMU陀螺仪, dt=0.05s
**输出**：更新全局位姿 (x, y, heading)

**算法**：
```cpp
// 1. 计算轮速
wheel_circ = π * WHEEL_DIAMETER;
v_left = (left_rpm / 60) * wheel_circ;
v_right = (right_rpm / 60) * wheel_circ;
v = (v_left + v_right) / 2;

// 2. 更新朝向（IMU积分）
gyro_z = imu_get_gyro_z();
gyro_sum += gyro_z * dt;
heading = normalize_angle(gyro_sum - heading_offset);

// 3. 更新位置
theta_rad = heading * π / 180;
dx = v * cos(theta_rad) * dt;
dy = v * sin(theta_rad) * dt;
pos_x += dx * 39.3701;  // 米转英寸
pos_y += dy * 39.3701;
```

---

### **4. ToF位置校正** (`correct_position_with_tof`)

**目的**：减少里程计累积误差

**算法**：
```cpp
if (tof_left > 0 && tof_left < 500) {
    measured_x = MM_TO_INCHES(tof_left) + ROBOT_RADIUS;
    // 平滑融合（70%里程计 + 30%ToF）
    pos_x = pos_x * 0.7 + measured_x * 0.3;
}
```

---

## 🔌 **API接口**

### **初始化**
```cpp
esp_err_t wall_following_init(void);
```
- 创建互斥锁
- 设置初始状态为 `WF_STATE_IDLE`

### **启动寻墙**
```cpp
esp_err_t wall_following_start(void);
```
- 启动异步ToF读取
- 创建寻墙任务（Core 1, 优先级5, 20Hz）
- 设置状态为 `WF_STATE_INIT_POS`
- **禁用手动控制**（`g_manual_control_enabled = false`）

### **停止寻墙**
```cpp
esp_err_t wall_following_stop(void);
```
- 设置状态为 `WF_STATE_STOPPED`
- 停止底盘（`chassis_v2_set_velocity(0, 0)`）
- 删除任务
- 停止异步ToF读取
- **恢复手动控制**（`g_manual_control_enabled = true`）

### **获取状态**
```cpp
esp_err_t wall_following_get_status(wall_follow_status_t *status);
```
返回：
- 当前状态
- ToF读数
- 朝向、累计距离
- 运行标志

---

## 🐛 **调试输出**

### **ToF读数**（每2秒）
```
[WALL_FOLLOW] [ToF] Front=XXX Left=XXX (raw: F=XXX L=XXX)
```

### **初始化**
```
[WALL_FOLLOW] [STATE] INIT_POS - Calculating initial position...
[WALL_FOLLOW] ToF avg: Left=XXX.Xmm, Front=XXX.Xmm
[WALL_FOLLOW] Init pos: (XX.XX, XX.XX), heading=90.0°
[WALL_FOLLOW] ✅ Init complete! Pos:(XX.X, XX.X) Heading:90.0°
[WALL_FOLLOW] Switching to FOLLOW_WALL state...
```

### **循墙控制**（每1秒）
```
[WALL_FOLLOW] [FOLLOW] Pos:(XX.X,XX.X) Head:XX.X° ToF:L=XX F=XX Angular:X.XXX
```

---





## ⚠️ **已知问题和限制**

### **1. 缺少转角检测**
**问题**：当前只有 `FOLLOW_WALL` 状态，没有转角检测和旋转逻辑。

**表现**：
- 遮挡前方ToF时，不会触发旋转
- 遇到墙角时会卡住

**需要添加的状态**：
```cpp
WF_STATE_TURN_CORNER  // 检测到墙角，执行90°旋转
```

**转角检测逻辑**（待实现）：
```cpp
if (tof_front < 200 && tof_left < 200) {
    // 检测到墙角
    state = WF_STATE_TURN_CORNER;
}
```

---

### **2. 手动控制冲突**
**问题**：如果 `g_manual_control_enabled = true`，Web摇杆会覆盖寻墙指令。

**解决方案**：
- `wall_following_start()` 自动设置 `g_manual_control_enabled = false`
- `wall_following_stop()` 自动恢复 `g_manual_control_enabled = true`

**检查方法**：
```cpp
// 在 wuer.ino 的 chassis_control_task 中
if (web_server_is_manual_control_enabled()) {
    // 只有手动模式才执行摇杆控制
    chassis_v2_set_velocity(linear, angular);
}
```

---

### **3. ToF读数无效**
**问题**：如果ToF传感器未初始化或读数无效，会卡在 `INIT_POS` 状态。

**调试方法**：
```
[WALL_FOLLOW] [ToF] Front=0 Left=0 (raw: F=65535 L=65535)
[WALL_FOLLOW] ❌ Init failed (ToF invalid), retrying in 500ms...
```

**解决方案**：
- 确保 `tof_start_async_reading()` 成功启动
- 检查I2C总线和TCA9548A多路复用器
- 确保车辆左侧和前方有墙壁（< 1000mm）

---

### **4. 里程计累积误差**
**问题**：长时间运行后，位置估计会漂移。

**当前缓解措施**：
- ToF位置校正（30%融合权重）
- 只校正X坐标（左墙距离）

**改进方向**：
- 增加前墙校正（Y坐标）
- 使用Vive定位系统融合
- 增加转角处的位置重置

---

### **5. 右轮电机震荡**
**问题**：右轮电机漏液导致震荡，影响里程计精度。

**当前解决方案**：
- 右轮单独PID参数：Kp=10.0, Ki=1.5, Kd=0.5
- 左轮PID参数：Kp=10.0, Ki=3.0, Kd=0.2

**影响评估**：
- ✅ 如果两轮RPM平均值接近目标（误差 < 15%），寻墙仍可工作
- ✅ 如果左右轮RPM差值稳定（震荡幅度 < 10 RPM），直线行驶不受影响
- ❌ 轮速波动会导致里程计误差累积

---

## 🔧 **集成说明**

### **依赖模块**
```cpp
#include "include/chassis_v2.h"   // 底盘控制（V2版本）
#include "include/tof_sensor.h"   // ToF传感器
#include "include/imu_sensor.h"   // IMU陀螺仪
#include "include/encoder.h"      // 编码器
```

### **初始化顺序**（在 `wuer.ino` 中）
```cpp
// 1. 硬件初始化
tof_init();
imu_init();
encoder_init();
chassis_v2_init();

// 2. 寻墙初始化
wall_following_init();
```

### **Web界面集成**
- **启动**：`/startWallFollow` → `wall_following_start()`
- **停止**：`/stopWallFollow` → `wall_following_stop()`
- **状态**：`/getWallFollowStatus` → `wall_following_get_status()`

---

## 📊 **性能指标**

### **控制频率**
- **主任务**：20Hz（50ms周期）
- **ToF读取**：异步缓存（20Hz）
- **底盘控制**：20Hz（Chassis V2内部）

### **内存占用**
- **任务栈**：8192 bytes
- **全局变量**：~100 bytes

### **CPU占用**
- **Core 1**：寻墙任务（优先级5）
- **预估负载**：< 10%（大部分时间在延时）

---

## 🚀 **使用示例**

### **启动寻墙**
```cpp
// 在Web界面点击 "Start Wall Follow"
// 或通过代码调用
esp_err_t ret = wall_following_start();
if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Wall following started");
}
```

### **监控状态**
```cpp
wall_follow_status_t status;
if (wall_following_get_status(&status) == ESP_OK) {
    printf("State: %d\n", status.state);
    printf("ToF: Front=%u Left=%u\n", status.tof_front, status.tof_left);
    printf("Heading: %.1f°\n", status.current_heading);
    printf("Distance: %.2fm\n", status.total_distance);
}
```

### **停止寻墙**
```cpp
// 在Web界面点击 "Stop Wall Follow"
// 或通过代码调用
wall_following_stop();
```

---

## 📝 **待办事项**

### **高优先级**
- [ ] 添加转角检测和旋转逻辑
- [ ] 测试初始位置计算的准确性
- [ ] 验证ToF异步读取的稳定性

### **中优先级**
- [ ] 增加前墙位置校正（Y坐标）
- [ ] 优化PID参数（减少震荡）
- [ ] 添加Vive定位融合

### **低优先级**
- [ ] 记录完整轨迹（用于调试）
- [ ] 添加路径规划（避障）
- [ ] 支持顺时针/逆时针选择

---

## 📚 **相关文档**

- **底盘控制**：`docs/chassis_backup_before_rebuild.md`
- **PID算法**：`src/pid_v2.cpp`
- **ToF传感器**：`src/tof_sensor.cpp`
- **Web界面**：`src/include/web.h`

---

**最后更新**：2025-12-09
**维护者**：MEAM 5100 Team
**状态**：🚧 开发中，核心功能已实现，转角逻辑待添加
