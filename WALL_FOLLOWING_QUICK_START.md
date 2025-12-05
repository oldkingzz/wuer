# 寻墙算法快速参考 / Wall Following Quick Reference

## 📁 文件信息

- **文件名**: `wall_following_test.ino`
- **总行数**: 564行
- **状态**: ✅ 已完成，可直接上传

---

## 🎯 核心特性

✅ **原地自旋转向** (Spin in Place) - 转弯半径最小  
✅ **凹角处理** - 检测墙角并左转90度  
✅ **凸角处理** - 检测Tower/Nexus并绕过  
✅ **PID循墙控制** - 保持恒定距离  
✅ **IMU精确转向** - 陀螺仪积分实现精确90度转向  

---

## 🔧 需要填写的参数

### **1. ToF阈值** (第37-40行)

```cpp
#define WALL_DISTANCE_TARGET        200     // 【填写】目标循墙距离 (mm)
#define FRONT_OBSTACLE_THRESHOLD    150     // 【填写】前方障碍物阈值 (mm)
#define SIDE_WALL_LOST_THRESHOLD    400     // 【填写】侧面墙壁消失阈值 (mm)
#define SIDE_WALL_FOUND_THRESHOLD   300     // 【填写】侧面墙壁检测阈值 (mm)
```

**推荐值**：
- 目标距离：150-250mm（根据机器人宽度）
- 前方阈值：100-200mm（越小越晚转向）
- 侧面消失：300-500mm（检测凸角灵敏度）

---

### **2. ToF传感器间距** (第43行)

```cpp
#define TOF_SIDE_SPACING            150     // 【填写】左右ToF之间的距离 (mm)
```

**说明**：预留参数，用于未来实现平行度检测

---

### **3. 运动参数** (第46-48行)

```cpp
#define FORWARD_SPEED               0.15f   // 【可选】前进速度 (m/s)
#define TURN_ANGULAR_SPEED          0.5f    // 【可选】转向角速度 (rad/s)
#define CLEARANCE_DISTANCE          0.15f   // 【可选】绕过凸角后的延迟距离 (m)
```

**推荐值**：
- 前进速度：0.1-0.3 m/s
- 转向速度：0.3-0.8 rad/s
- 延迟距离：根据机器人长度（约0.1-0.2m）

---

### **4. PID参数** (第51-53行)

```cpp
#define WALL_FOLLOW_KP              0.001f  // 【可选】比例增益
#define WALL_FOLLOW_KI              0.0f    // 【可选】积分增益
#define WALL_FOLLOW_KD              0.0001f // 【可选】微分增益
```

**调试技巧**：
- 摆动太大 → 减小Kp
- 反应太慢 → 增大Kp
- 稳态误差 → 增大Ki

---

## 📊 状态机流程

```
START
  ↓
FIND_WALL (原地旋转寻墙)
  ↓
FOLLOW_WALL (循墙前进)
  ↓
  ├─→ 前方障碍物 → TURN_AWAY (左转90°) → FOLLOW_WALL
  │
  └─→ 侧面墙壁消失 → CLEARANCE (直行延迟)
                       ↓
                    TURN_TOWARD (右转90°)
                       ↓
                    FIND_WALL (寻找墙壁)
                       ↓
                    FOLLOW_WALL
```

---

## 🚀 使用步骤

### **1. 上传程序**
```bash
Arduino IDE → 选择ESP32S3 → 编译上传
```

### **2. 打开串口监视器**
- 波特率：115200
- 观察初始化输出

### **3. 放置机器人**
- 放在场地中央
- 右侧有墙壁（距离<300mm）

### **4. 观察行为**
- 机器人会自动寻墙并开始循墙

---

## 📺 串口输出示例

### **初始化**
```
========================================
Wall Following Test Program
========================================
Step 1/7: Initializing I2C bus...
OK: I2C initialized
...
========================================
Wall Following Started!
========================================
```

### **运行时状态** (每2秒)
```
========================================
State: FOLLOW_WALL
ToF - Front: 450 mm, Left: 0 mm, Right: 205 mm
IMU - Heading: 12.3°
Distance Traveled: 1.25 m
========================================
```

---

## 🐛 常见问题

| 问题 | 解决方案 |
|------|---------|
| 机器人不转向 | 增大`FRONT_OBSTACLE_THRESHOLD` |
| 机器人离墙太近 | 增大`WALL_DISTANCE_TARGET` |
| 机器人摆动 | 减小`WALL_FOLLOW_KP` |
| 转向不准确 | 检查IMU是否正常 |
| 绕不过Tower | 增大`CLEARANCE_DISTANCE` |

---

## 📋 传感器索引

```cpp
g_tof_front = tof_read_distance(0);  // 前方ToF
g_tof_left  = tof_read_distance(1);  // 左侧ToF
g_tof_right = tof_read_distance(2);  // 右侧ToF
```

**确保你的ToF传感器按此顺序连接！**

---

## ⚙️ 算法逻辑

### **凹角（墙角）**
```
前方ToF < 150mm → 停止 → 左转90° → 继续循墙
```

### **凸角（Tower/Nexus）**
```
侧面ToF > 400mm → 直行0.15m → 右转90° → 寻墙 → 继续循墙
```

---

## ✅ 检查清单

在上传程序前，确认：

- [ ] ToF传感器已正确连接（索引0=前，1=左，2=右）
- [ ] IMU已初始化并能读取陀螺仪数据
- [ ] 电机驱动器已连接电源（7-12V）
- [ ] 编码器已正确连接
- [ ] 已填写所有必要参数（阈值、间距等）

---

## 📖 详细文档

完整的算法说明和调试指南请查看：**`WALL_FOLLOWING_README.md`**

---

**程序已准备就绪，可以直接上传测试！** 🎉

**重要提醒**：
1. 这是一个**独立的测试程序**，不会修改你的主程序 `wuer.ino`
2. 所有参数都可以在文件顶部轻松修改
3. 阈值和距离需要根据实际场地调整

