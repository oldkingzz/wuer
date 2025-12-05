# 寻墙算法测试程序 / Wall Following Test Program

## 概述 / Overview

这是一个独立的寻墙测试程序，专为差速轮机器人设计，能够处理凹角（墙角）和凸角（Tower/Nexus）。

**文件名**: `wall_following_test.ino`

---

## 算法特性 / Algorithm Features

### ✅ **核心功能**

1. **状态机控制** - 6个状态处理不同场景
2. **原地自旋转向** - Spin in Place，转弯半径最小
3. **PID循墙控制** - 保持与墙壁的恒定距离
4. **凹角处理** - 检测墙角并转离墙壁
5. **凸角处理** - 检测Tower/Nexus并绕过
6. **IMU精确转向** - 使用陀螺仪积分实现精确90度转向

---

## 状态机 / State Machine

```
┌─────────────┐
│ FIND_WALL   │ ← 初始状态：原地旋转寻找墙壁
└──────┬──────┘
       │ 检测到右侧墙壁
       ↓
┌─────────────┐
│ FOLLOW_WALL │ ← 循墙前进（PID控制）
└──────┬──────┘
       │
       ├─→ 前方障碍物 → TURN_AWAY（凹角）
       │
       └─→ 侧面墙壁消失 → CLEARANCE（凸角）

【凹角处理流程】
TURN_AWAY → FOLLOW_WALL
（左转90度，远离墙壁）

【凸角处理流程】
CLEARANCE → TURN_TOWARD → FIND_WALL → FOLLOW_WALL
（直行延迟 → 右转90度 → 寻找墙壁 → 继续循墙）
```

---

## 传感器配置 / Sensor Configuration

### **ToF传感器** (3个VL53L0X)

- **索引0**: 前方ToF - 检测前方障碍物（凹角）
- **索引1**: 左侧ToF - 备用
- **索引2**: 右侧ToF - 循墙距离控制

### **IMU传感器** (MPU6050)

- **Z轴陀螺仪**: 朝向角度积分（精确转向）

### **编码器** (2个)

- **用途**: 计算累计行驶距离（用于凸角延迟直行）

---

## 可配置参数 / Configurable Parameters

在 `wall_following_test.ino` 文件顶部修改以下参数：

### **1. ToF阈值** (单位: mm)

```cpp
#define WALL_DISTANCE_TARGET        200     // 目标循墙距离
#define FRONT_OBSTACLE_THRESHOLD    150     // 前方障碍物阈值
#define SIDE_WALL_LOST_THRESHOLD    400     // 侧面墙壁消失阈值
#define SIDE_WALL_FOUND_THRESHOLD   300     // 侧面墙壁检测阈值
```

**调整建议**：
- `WALL_DISTANCE_TARGET`: 根据你的机器人宽度和场地大小调整（推荐150-250mm）
- `FRONT_OBSTACLE_THRESHOLD`: 越小越晚转向，越大越早转向（推荐100-200mm）
- `SIDE_WALL_LOST_THRESHOLD`: 检测凸角的灵敏度（推荐300-500mm）

---

### **2. 运动参数**

```cpp
#define FORWARD_SPEED               0.15f   // 前进速度 (m/s)
#define TURN_ANGULAR_SPEED          0.5f    // 转向角速度 (rad/s)
#define CLEARANCE_DISTANCE          0.15f   // 绕过凸角后的延迟距离 (m)
```

**调整建议**：
- `FORWARD_SPEED`: 根据场地大小和电机功率调整（推荐0.1-0.3 m/s）
- `TURN_ANGULAR_SPEED`: 转向速度，越大转得越快（推荐0.3-0.8 rad/s）
- `CLEARANCE_DISTANCE`: 根据机器人长度调整，确保车身完全越过障碍物

---

### **3. PID参数** (循墙距离控制)

```cpp
#define WALL_FOLLOW_KP              0.001f  // 比例增益
#define WALL_FOLLOW_KI              0.0f    // 积分增益
#define WALL_FOLLOW_KD              0.0001f // 微分增益
```

**调整建议**：
- 如果机器人摆动太大 → 减小Kp
- 如果机器人反应太慢 → 增大Kp
- 如果有稳态误差 → 增大Ki（小心积分饱和）
- 如果震荡 → 增大Kd

---

### **4. IMU转向精度**

```cpp
#define TURN_ANGLE_TOLERANCE        2.0f    // 转向角度容差 (度)
```

**调整建议**：
- 越小越精确，但可能导致转向时间过长
- 推荐1-5度

---

### **5. ToF传感器间距** (重要！)

```cpp
#define TOF_SIDE_SPACING            150     // 【用户填写】左右ToF之间的距离 (mm)
```

**说明**：
- 这个参数目前未使用，但预留用于检测机器人是否平行于墙壁
- 如果需要实现平行度检测，可以比较左右ToF的读数差值

---

## 使用方法 / Usage

### **步骤1: 上传程序**

1. 打开Arduino IDE
2. 加载 `wall_following_test.ino`
3. 选择开发板：ESP32S3 Dev Module
4. 编译并上传

---

### **步骤2: 打开串口监视器**

- 波特率：115200
- 观察初始化过程

**期望输出**：
```
========================================
Wall Following Test Program
========================================
Step 1/7: Initializing I2C bus...
OK: I2C initialized
Step 2/7: Initializing motor drivers...
OK: Motor drivers initialized
...
========================================
Wall Following Started!
========================================
```

---

### **步骤3: 放置机器人**

1. 将机器人放在场地中央
2. 确保右侧有墙壁（距离<300mm）
3. 机器人会自动开始寻墙

---

### **步骤4: 观察行为**

**正常流程**：
1. **FIND_WALL**: 原地旋转，寻找右侧墙壁
2. **FOLLOW_WALL**: 沿墙前进，保持200mm距离
3. **遇到墙角**: 
   - 前方ToF检测到障碍物
   - 切换到TURN_AWAY
   - 左转90度
   - 继续FOLLOW_WALL
4. **遇到Tower/Nexus**:
   - 侧面ToF读数突然变大
   - 切换到CLEARANCE
   - 直行0.15m
   - 切换到TURN_TOWARD
   - 右转90度
   - 切换到FIND_WALL
   - 重新找到墙壁后继续FOLLOW_WALL

---

### **步骤5: 调试**

**串口输出**（每2秒）：
```
========================================
State: FOLLOW_WALL
ToF - Front: 450 mm, Left: 0 mm, Right: 205 mm
IMU - Heading: 12.3°
Distance Traveled: 1.25 m
========================================
```

**调试技巧**：
- 如果机器人不转向 → 检查`FRONT_OBSTACLE_THRESHOLD`是否太小
- 如果机器人转向太早 → 增大`FRONT_OBSTACLE_THRESHOLD`
- 如果机器人离墙太近/太远 → 调整`WALL_DISTANCE_TARGET`
- 如果机器人摆动 → 调整PID参数
- 如果转向不准确 → 检查IMU是否正常工作

---

## 算法逻辑详解 / Algorithm Details

### **凹角处理（Inner Corner）**

```
机器人沿墙前进
    │
    ↓
前方ToF < 150mm（检测到墙）
    │
    ↓
停止前进
    │
    ↓
原地左转90度（远离墙壁）
    │
    ↓
继续循墙前进
```

---

### **凸角处理（Outer Corner - Tower/Nexus）**

```
机器人沿墙前进
    │
    ↓
侧面ToF > 400mm（墙壁消失）
    │
    ↓
直行0.15m（确保车身越过障碍物）
    │
    ↓
原地右转90度（转向墙壁）
    │
    ↓
原地旋转寻找墙壁
    │
    ↓
检测到墙壁后继续循墙前进
```

---

## 注意事项 / Notes

1. **电机电源**: 确保L298N的VCC端子有7-12V电源
2. **ToF传感器高度**: 确保ToF能照到Nexus/Tower（高度匹配）
3. **IMU校准**: 程序启动时会自动归零IMU朝向
4. **编码器方向**: 如果距离计算错误，检查编码器接线
5. **差速轮特性**: 原地旋转时左右轮速度相反

---

## 故障排除 / Troubleshooting

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 机器人不转向 | ToF阈值太小 | 增大`FRONT_OBSTACLE_THRESHOLD` |
| 机器人离墙太近 | 目标距离太小 | 增大`WALL_DISTANCE_TARGET` |
| 机器人摆动 | PID参数不合适 | 减小Kp，增大Kd |
| 转向不准确 | IMU漂移 | 检查IMU初始化，增加校准时间 |
| 绕不过Tower | 延迟距离太短 | 增大`CLEARANCE_DISTANCE` |

---

## 下一步优化 / Future Improvements

1. **双侧ToF平行度检测**: 使用左右ToF差值判断是否平行于墙壁
2. **动态速度调整**: 根据前方距离动态调整速度
3. **多圈计数**: 记录循墙圈数，自动停止
4. **障碍物分类**: 区分墙角和Tower，采用不同策略

---

**程序已准备就绪，可以直接上传测试！** 🚀
