# 当前导航 & Mission 已完成功能概览（Step0–Step3）

> 本文是配合 `navigation_mission_plan.md` 使用的 **实际实现记录**，方便你后面查找改哪些地方。

## 1. Vive 异步读取改造（任务读取）

相关文件：
- `src/include/vive_sensor.h`
- `src/vive_sensor.cpp`
- `wuer.ino`

已经完成：
- 在 `vive_sensor` 模块内部增加 **FreeRTOS 异步读取任务**：
  - 新接口：`esp_err_t vive_start_async_reading(void);`
  - 停止接口：`void vive_stop_async_reading(void);`
  - 缓存读取接口：`esp_err_t vive_get_latest_all(vive_data_t *sensor1_data, vive_data_t *sensor2_data);`
- 在 `wuer.ino` 里，在 `vive_init()` 之后调用 `vive_start_async_reading()`：
  - 整个系统不再在高层直接 `vive_read_all()` 做周期采集，统一从缓存拿数据。

以后如果要改：
- Vive 读取频率、任务优先级等：去 `src/vive_sensor.cpp` 里改异步任务参数即可。

---

## 2. 中央导航配置模块 `nav_config`（Step0 & Step1）

相关文件：
- `src/include/nav_config.h`
- `src/nav_config.cpp`

已经完成：
1. **6 个预设导航目标点**（地图坐标 + 期望朝向）
   - 结构体：`nav_goal_t { int16_t map_x, map_y; float heading_deg; }`
   - 枚举：`nav_goal_id_t { NAV_GOAL_0 ... NAV_GOAL_5 }`
   - 数据：`g_nav_goals[NAV_NUM_GOALS]`（`nav_config.cpp` 中目前是占位值）

2. **对应的 12 个 Vive 实测坐标占位值**
   - 结构体：`nav_goal_vive_sample_t { vive1_x, vive1_y, vive2_x, vive2_y }`
   - 数据：`g_nav_goal_vive_samples[NAV_NUM_GOALS]`（`nav_config.cpp` 中为占位值）

3. **低层导航参数（原来散落在 `vive_navigation.h` 里的 NAV_*）**
   - 统一放在 `nav_config.h`：
     - `NAV_ARRIVAL_THRESHOLD`
     - `NAV_HEADING_THRESHOLD`
     - `NAV_MAX_LINEAR_VELOCITY`
     - `NAV_MAX_ANGULAR_VELOCITY`
     - `NAV_LOOKAHEAD_DISTANCE`
     - `NAV_REPLAN_THRESHOLD`
     - `NAV_DISTANCE_KP`
     - `NAV_HEADING_KP`

4. **Mission 用的高层参数 NAV_MISSION_*（撞击任务相关）**
   - GOTO_PRE_POINT：
     - `NAV_MISSION_GOTO_PREPOINT_TIMEOUT_MS`
   - ALIGN_TOF：
     - `NAV_MISSION_ALIGN_FRONT_TARGET_MM`
     - `NAV_MISSION_ALIGN_FRONT_TOL_MM`
     - `NAV_MISSION_ALIGN_SIDE_TARGET_MM`
     - `NAV_MISSION_ALIGN_SIDE_TOL_MM`
     - `NAV_MISSION_ALIGN_ANGULAR_SPEED`
     - `NAV_MISSION_ALIGN_STABLE_TIME_MS`
     - `NAV_MISSION_ALIGN_MAX_TIME_MS`
   - FORWARD_IMPACT：
     - `NAV_MISSION_IMPACT_FORWARD_SPEED`
     - `NAV_MISSION_IMPACT_MAX_TRAVEL_MM`
     - `NAV_MISSION_IMPACT_FRONT_STOP_MM`
     - `NAV_MISSION_IMPACT_MAX_TIME_MS`
   - RETURN：
     - `NAV_MISSION_RETURN_SPEED`
     - `NAV_MISSION_RETURN_TRAVEL_MM`
     - `NAV_MISSION_RETURN_MAX_TIME_MS`
   - 通用：
     - `NAV_MISSION_TOF_FRESH_TIMEOUT_MS`

以后如果要改：
- **所有导航/任务相关的「数字」优先改这里**（阈值、速度、距离、时间等）。
- 6 个目标点的位置 & 朝向：改 `g_nav_goals[]`。
- 6 组 Vive 原始坐标样本：改 `g_nav_goal_vive_samples[]`。

---

## 3. `vive_navigation` 改造使用 `nav_config`（Step2）

相关文件：
- `src/include/vive_navigation.h`
- `src/vive_navigation.cpp`

已经完成：
1. `vive_navigation.h` 不再定义任何 `NAV_*` 宏：
   - 统一通过 `#include "nav_config.h"` 获取导航参数。

2. 新增按「目标点 ID」设目标的封装：
   - 接口：`esp_err_t vive_nav_set_target_goal(nav_goal_id_t goal_id);`
   - 实现：内部从 `g_nav_goals[goal_id]` 取地图坐标，然后调用 `vive_nav_set_target_map(map_x, map_y)`。

3. 原有低层导航 API 保持不变：
   - `vive_nav_init()`
   - `vive_nav_set_target(int16_t x, int16_t y)` / `vive_nav_set_target_map(...)`
   - `vive_nav_start()` / `vive_nav_stop()` / `vive_nav_replan()`
   - `vive_nav_get_status()` / `vive_nav_get_pose()` / `vive_nav_get_path()`

4. 导航内部控制逻辑未改，只是参数来源统一为 `nav_config.h`：
   - 使用 `NAV_DISTANCE_KP` / `NAV_HEADING_KP` 做比例控制；
   - 使用 `NAV_MAX_LINEAR_VELOCITY` / `NAV_MAX_ANGULAR_VELOCITY` 做限幅；
   - 使用 `NAV_ARRIVAL_THRESHOLD` 判断到达；
   - 使用 `NAV_REPLAN_THRESHOLD` 触发重规划。

以后如果要改：
- 只改参数时，不需要动 `vive_navigation.cpp`，直接改 `nav_config.h/.cpp` 即可。
- 如果要增加新的「点类型」（比如更多预备点）：扩展 `nav_goal_id_t` 和 `g_nav_goals[]`。

---

## 4. Mission 状态机实现（Step3，尚未接入 Web）

相关文件：
- `src/include/vive_navigation.h`
- `src/vive_navigation.cpp`
- 使用的传感器/底盘模块：`tof_sensor`、`chassis`、`vive_sensor`

### 4.1 对外 Mission API

在 `vive_navigation.h` 中新增类型 & API：
- 状态枚举：`nav_mission_state_t`
  - `NAV_MISSION_STATE_IDLE`
  - `NAV_MISSION_STATE_GOTO_PRE_POINT`
  - `NAV_MISSION_STATE_ALIGN_TOF`
  - `NAV_MISSION_STATE_FORWARD_IMPACT`
  - `NAV_MISSION_STATE_RETURN`
  - `NAV_MISSION_STATE_DONE`
  - `NAV_MISSION_STATE_ERROR`
- 状态结构体：`nav_mission_status_t { state, goal_id, nav_state, state_elapsed_ms }`
- API：
  - `esp_err_t nav_mission_start(nav_goal_id_t goal_id);`
  - `esp_err_t nav_mission_stop(void);`
  - `esp_err_t nav_mission_get_status(nav_mission_status_t *status);`

### 4.2 内部结构

在 `src/vive_navigation.cpp` 中：
- 增加 Mission 状态全局变量：
  - `g_mission_state` / `g_mission_goal_id` / `g_mission_active`
  - 若干 Tick 计时变量（`g_mission_state_start_tick` 等）
  - 距离积分变量：`g_impact_travel_mm` / `g_return_travel_mm` 等
- 新建 Mission 专用任务：`mission_task`（20Hz）
  - 负责驱动状态机：
    - `GOTO_PRE_POINT`：监督低层 `vive_nav_*` 到达预备点
    - `ALIGN_TOF`：用 ToF 对墙或按钮对齐
    - `FORWARD_IMPACT`：直线前进到接触/足够近
    - `RETURN`：后退到安全位置

### 4.3 各阶段逻辑概述

- **GOTO_PRE_POINT**：
  - `nav_mission_start()` 时：
    - 调 `vive_nav_set_target_goal(goal_id)` + `vive_nav_start()`；
  - 在 Mission 任务中：
    - 使用 `vive_nav_get_status()` 监控 `NAV_STATE_ARRIVED`；
    - 若在 `NAV_MISSION_GOTO_PREPOINT_TIMEOUT_MS` 内到达 → 进入 `ALIGN_TOF`；
    - 否则超时 → `ERROR`。

- **ALIGN_TOF**：
  - 使用 ToF 缓存接口：
    - `tof_get_cached_front_distance()`
    - `tof_get_cached_left_front_distance()`
    - `tof_is_data_fresh(TOF_FRONT / TOF_LEFT_FRONT, NAV_MISSION_TOF_FRESH_TIMEOUT_MS)`
  - 目标：前后/侧向距离落在 `NAV_MISSION_ALIGN_*` 设定窗口，并稳定 `NAV_MISSION_ALIGN_STABLE_TIME_MS`；
  - 控制：
    - 若不在窗口内：根据侧向误差方向，`chassis_set_velocity(0, ±NAV_MISSION_ALIGN_ANGULAR_SPEED)` 原地旋转；
    - 若在窗口且稳定：停车，进入 `FORWARD_IMPACT`；
    - 若总时间超过 `NAV_MISSION_ALIGN_MAX_TIME_MS`：`ERROR`。

- **FORWARD_IMPACT**：
  - 按 `NAV_MISSION_IMPACT_FORWARD_SPEED` 匀速前进；
  - 用 `speed * dt` 积分得到 `g_impact_travel_mm`，并用前方 ToF 判断「到位」：
    - 若 `front_toF <= NAV_MISSION_IMPACT_FRONT_STOP_MM`：认为撞到/接触，停车，进入 `RETURN`；
    - 若时间超过 `NAV_MISSION_IMPACT_MAX_TIME_MS` 或距离超过 `NAV_MISSION_IMPACT_MAX_TRAVEL_MM`：`ERROR`。

- **RETURN**：
  - 以 `NAV_MISSION_RETURN_SPEED` 倒车；
  - 同样用积分方式估算后退距离 `g_return_travel_mm`；
  - 达到 `NAV_MISSION_RETURN_TRAVEL_MM` 或超时 `NAV_MISSION_RETURN_MAX_TIME_MS`：停车，状态置为 `DONE`。

> 注意：目前 RETURN 阶段只是按时间/速度积分估算距离，没有用到编码器，可后续改进。

---

## 5. 快速使用 & 调参入口

1. **启动 Mission（不经过 Web，直接在代码里测试）**
   - 保证系统已初始化（`vive_nav_init` / `vive_start_async_reading` / `tof_start_async_reading` / `chassis` 等）；
   - 调用：`nav_mission_start(NAV_GOAL_0);` 等；
   - 用 `nav_mission_get_status()` 查询当前状态。

2. **强制停止 Mission**
   - 随时调用：`nav_mission_stop();`（会停导航 + 底盘）。

3. **以后要改的主要地方**
   - 调整任务行为（距离、速度、时间、阈值）：`src/include/nav_config.h` 中的 `NAV_*` / `NAV_MISSION_*`；
   - 调整 6 个目标点（pre point/按钮点布置）：`src/nav_config.cpp` 中的 `g_nav_goals[]`；
   - 写入实测 Vive 标定样本：`src/nav_config.cpp` 中的 `g_nav_goal_vive_samples[]`；
   - 若要增加新的 Mission 流程（比如多次撞击）：在 `vive_navigation.cpp` 的 Mission 状态机基础上扩展新的 state 即可。

