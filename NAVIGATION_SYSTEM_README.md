# A* Navigation System Implementation

## 概述 / Overview

已成功实现基于A*算法的路径规划导航系统，集成Vive定位和Web地图界面。

Successfully implemented A* path planning navigation system with Vive localization and web map interface.

---

## 新增文件 / New Files

### 1. **src/include/grid_map.h** (150行)
- 栅格地图定义
- ROBA场地规格：144" x 60"，分辨率2像素/英寸
- 地图尺寸：288 x 120像素
- Cost值定义：
  - `COST_FREE = 1` (平地)
  - `COST_RAMP = 2` (坡道)
  - `COST_TOWER = 5` (塔)
  - `COST_OBSTACLE = 255` (障碍物/墙)

### 2. **src/grid_map.cpp** (203行)
- 地图初始化函数
- 绘制墙壁、Nexus基地、塔、坡道
- 坐标转换函数：
  - Vive坐标 ↔ 像素坐标
  - 英寸坐标 ↔ 像素坐标

### 3. **src/include/astar.h** (115行)
- A*算法接口定义
- 路径数据结构 `path_t`
- 最大路径长度：500个路径点
- 最大迭代次数：10,000次

### 4. **src/astar.cpp** (420行)
- A*路径规划算法实现
- 8方向连通（支持对角线移动）
- 路径简化算法（移除冗余点）
- 前瞻控制（lookahead）
- 路径有效性检查

### 5. **src/vive_navigation.cpp** (425行) - **重写**
- 导航状态机：
  - `NAV_STATE_IDLE` - 空闲
  - `NAV_STATE_PLANNING` - 路径规划中
  - `NAV_STATE_NAVIGATING` - 导航中
  - `NAV_STATE_ARRIVED` - 已到达
  - `NAV_STATE_ERROR` - 错误
- PID控制器（距离和朝向）
- 自动重规划（偏离路径时）
- 20Hz导航任务

### 6. **src/include/vive_navigation.h** (209行) - **更新**
- 新增API函数：
  - `vive_nav_set_target_map()` - 设置地图坐标目标
  - `vive_nav_get_path()` - 获取当前路径
  - `vive_nav_replan()` - 强制重规划

---

## 修改文件 / Modified Files

### 1. **src/include/web.h**
- 添加导航地图Canvas (576x240像素)
- 地图点击交互
- 实时显示机器人位置和路径
- 导航状态显示

### 2. **src/html510.cpp**
- 新增HTTP端点：
  - `GET /setNavGoal?x=<map_x>&y=<map_y>` - 设置导航目标
  - `GET /getNavStatus` - 获取导航状态（JSON）
  - `GET /stopNav` - 停止导航
- 包含 `vive_navigation.h`

---

## 地图规格 / Map Specifications

### 场地元素 / Field Elements

1. **墙壁 (Walls)**: 1英寸厚，围绕整个场地
2. **Blue Nexus**: 8" x 5"，左侧
3. **Red Nexus**: 8" x 5"，右侧
4. **Tower 1**: 8" x 6"，位于 (40", 15")，平地上
5. **Tower 2**: 8" x 6"，位于 (100", 40")，坡道上
6. **坡道 (Ramp)**: X范围 20"-124"，Y范围 20"-40"
7. **坡道护栏**: 1英寸厚，Y=20" 和 Y=40"

### 坐标系统 / Coordinate Systems

- **Vive坐标**: 0-8191 (用户负责坐标变换)
- **地图像素坐标**: 0-287 (X), 0-119 (Y)
- **物理坐标**: 0-144英寸 (X), 0-60英寸 (Y)

---

## 使用方法 / Usage

### 1. 初始化导航系统

在 `wuer.ino` 的 `setup()` 中添加：

```cpp
// 在Vive传感器初始化之后
ret = vive_nav_init();
if (ret != ESP_OK) {
    Serial.println("ERROR: Navigation init failed!");
    while(1) { delay(1000); }
}
Serial.println("OK: Navigation initialized");
```

### 2. Web界面使用

1. 连接到ESP32 WiFi热点：`ESP32_Chassis` (密码: `12345678`)
2. 打开浏览器访问：`http://192.168.4.1`
3. 在地图上点击任意位置设置导航目标
4. 机器人将自动规划路径并导航
5. 实时显示：
   - 绿色圆点：机器人当前位置
   - 红色圆点：目标位置
   - 蓝色线：规划的路径

### 3. 编程接口

```cpp
// 设置目标（地图坐标）
vive_nav_set_target_map(144, 60);  // 像素坐标

// 或设置目标（Vive坐标）
vive_nav_set_target(4096, 4096);  // Vive坐标

// 开始导航
vive_nav_start();

// 获取状态
nav_status_t status;
vive_nav_get_status(&status);

// 停止导航
vive_nav_stop();

// 强制重规划
vive_nav_replan();
```

---

## 导航参数 / Navigation Parameters

可在 `src/include/vive_navigation.h` 中调整：

- `NAV_ARRIVAL_THRESHOLD = 10` 像素 (5英寸)
- `NAV_LOOKAHEAD_DISTANCE = 20` 像素 (10英寸)
- `NAV_REPLAN_THRESHOLD = 50` 像素 (25英寸)
- `NAV_MAX_LINEAR_VELOCITY = 0.3` m/s
- `NAV_MAX_ANGULAR_VELOCITY = 1.0` rad/s
- 比例控制增益：
  - 距离控制：Kp=0.01 (速度 = 距离 × Kp)
  - 朝向控制：Kp=0.02 (角速度 = 角度误差 × Kp)

---

## 坡道必经点 / Mandatory Ramp Waypoints

用户提到"坡道上有两点我是一定要到的"，但未指定具体坐标。

如需实现多路径点导航，请提供两个必经点的坐标（英寸或像素），我将添加多段路径规划功能。

---

## 注意事项 / Notes

1. **Vive坐标变换**: 用户需要自行实现Vive坐标系到地图坐标系的变换
2. **电机电源**: 确保L298N的VCC端子有7-12V电源
3. **传感器校准**: 确保双Vive传感器正常工作
4. **内存使用**: A*节点池占用约200KB RAM
5. **任务优先级**: 导航任务优先级为5，运行在Core 1

---

## 编译和上传 / Build and Upload

系统已准备就绪，可以直接编译上传到ESP32-S3。

所有修改仅限于导航模块和Web模块，未修改其他代码。

---

## 下一步 / Next Steps

1. 测试Vive定位精度
2. 调整PID参数以优化路径跟踪
3. 添加多路径点导航（如需要）
4. 集成ToF传感器进行动态避障（可选）


