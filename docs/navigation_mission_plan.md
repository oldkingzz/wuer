## 导航与撞击任务设计总览

本文件只做**规划与任务拆解**，当前阶段不修改任何代码。目标：

- 所有导航相关的数值（阈值、速度、6 个目标点坐标等）集中在一个独立的 `*.h` 配置文件中；
- 在现有 Vive+A* 导航基础上，增加“到预备点 → ToF 对齐 → 直线撞击 → 返回”的高层任务状态机；
- 复用现有底盘和 ToF 接口，不修改底层实现和“写死”的速度配置，只在高层按需调用。

---

## 一、现有代码勘察结论（只读、不改）

1. **底盘接口**（可用，保持不变）
   - `src/include/chassis.h` / `chassis_v2.*`
     - 使用 `chassis_set_velocity(linear, angular)` 或 `chassis_v2_set_velocity(...)` 控制底盘。
     - 最大速度等在这里"写死"，本轮不改数值，只在高层选择合适线/角速度。

2. **ToF 接口**（可用，保持不变）
   - `src/include/tof_sensor.h`
   - 将使用的主要函数：
     - `tof_get_cached_front_distance()` 前方距离；
     - `tof_get_cached_left_front_distance()` 侧前距离；
     - 视需要也可用 `tof_get_left_rear_distance()`。

3. **墙循迹模块**（保持现有行为）
   - `src/include/wall_following.h`, `src/wall_following.cpp`
   - 已实现：起点 ToF 定位 + 预定义一圈轨迹 + 自动停止；
   - 后续只在模式切换时“启动/停止”，不改内部细节。

4. **栅格地图 & A***（可用，保持不变）
   - `grid_map.h/.cpp`：定义场地 144"x60"，1 像素/英寸，含 Nexus/Tower/Ramp 障碍；
   - `astar.h/.cpp`：通用 A* 规划 + 路径简化 + 前瞻点选取。

5. **Vive 传感与导航**
   - `vive_sensor.h`：底层两路 Vive 读数接口（坐标、valid、status）。
   - `vive_navigation.h/.cpp`：
     - 已有：
       - `vive_nav_set_target_map(map_x, map_y)` + `vive_nav_start()` → 调 A* + 按路径跟随；
       - `vive_nav_get_status()` / `vive_nav_get_path()` / `vive_nav_stop()`；
     - 当前直接通过 `grid_map_vive_to_pixel()` 建立 Vive→地图的线性映射。

6. **Web 服务器 & UI**
   - `src/html510.cpp`：HTTP handler 已有：
     - `/setNavGoal` → `vive_nav_set_target_map` + `vive_nav_start`；
     - `/getNavStatus` → 导出当前位置、目标、路径；
     - `/stopNav`、`/startWallFollow`、`/stopWallFollow` 等。
   - `src/include/web.h`：
     - Web 页面里已有 **导航地图 canvas**，点击地图→`/setNavGoal?x=..&y=..`；
     - 目前**没有** 6 个预设目标按钮，也没有“撞击任务”按钮和状态显示。

---

## 二、规划新增的导航配置头文件（集中所有“导航数值”）

> 仅规划：暂定文件名 `src/include/nav_config.h`，后续按此实现。

1. **集中导航阈值 & 参数**
   - 从 `vive_navigation.h` 中迁移或重定义：
     - `NAV_ARRIVAL_THRESHOLD`、`NAV_HEADING_THRESHOLD`；
     - `NAV_MAX_LINEAR_VELOCITY`、`NAV_MAX_ANGULAR_VELOCITY`；
     - `NAV_LOOKAHEAD_DISTANCE`、`NAV_REPLAN_THRESHOLD`；
     - `NAV_DISTANCE_KP`、`NAV_HEADING_KP` 等。
   - 同时新增：
     - ToF 对齐阶段用的前/侧距离期望区间（粗略先拟定一个安全范围）；
     - 直线撞击阶段的最大前进距离 / 时间上限；
     - 返回阶段的后退距离等。

2. **六个导航目标点配置**
   - 设计一个目标点结构，例如：
     - `map_x, map_y`（地图像素坐标，与你量出来的英寸坐标一一对应）；
     - `heading_deg`（在预备点期望的车头朝向）；
     - 可预留 `impact_offset` 或第二个“撞击点”坐标字段。
   - 提供 `extern` 的 `g_nav_goals[6]` 声明；
   - `.cpp` 里给出**占位默认值**（方便你明天量完直接改数值，不动逻辑）。

---

## 三、Vive 导航扩展与高层任务状态机设计（不破坏现有 API）

1. **保持现有底层 `vive_nav_*` 行为**
   - 继续使用当前 `vive_nav_set_target_map` + `vive_nav_start` + `vive_nav_get_status`；
   - 不改变内部 20Hz task 的基本结构和 A* 使用方式。

2. **在其之上增加“任务状态机”层（Mission FSM）**
   - 新建一个高层模块（可能在 `vive_navigation.cpp` 内或独立 `mission_nav.*`）：
     - `MISSION_IDLE`
     - `MISSION_GOTO_PRE_POINT`（调用 Vive 导航到预备点）
     - `MISSION_ALIGN_TOF`（原地慢速旋转，使用侧/前 ToF 找到合适几何关系）
     - `MISSION_FORWARD_IMPACT`（按现有底盘接口直线前进，直到 ToF/距离条件满足或超时）
     - `MISSION_RETURN`（按固定距离或 ToF 条件后退到安全位置）
     - `MISSION_DONE` / `MISSION_ERROR`
   - 每个 Web 按钮触发一个任务：选择 `g_nav_goals[i]` 对应配置并进入 `MISSION_GOTO_PRE_POINT`。

3. **ToF 对齐与撞击阶段的具体策略（参数由 nav_config.h 给出）**
   - `ALIGN_TOF`：
     - 使用 `tof_get_cached_front_distance()` + `tof_get_cached_left_front_distance()`；
     - 控制：`chassis_set_velocity(0, small_angular)` 左/右慢转；
     - 判定：在设定最大时间内，找到一段连续若干周期内（防抖）ToF 落在配置的区间内 → 认为姿态对齐。
   - `FORWARD_IMPACT`：
     - 控制：`chassis_set_velocity(v_forward, 0)`，速度 v_forward 选取不大于底盘上限；
     - 判定：
       - 前 ToF 小于某阈值（接近目标）或
       - 里程计累积前进距离达到配置上限 → 停止并进入 `MISSION_RETURN` 或 `DONE`。
   - `RETURN`：
     - 按记录的前进距离 / 固定距离后退；
     - 或以 ToF 恢复到预备点几何关系为准停止。

4. **模式互斥与安全**
   - 任务状态机需要确认当前不是 Wall Follow / 手动模式在占用底盘：
     - 复用 `html510.cpp` 中的 `g_manual_control_enabled` / Wall-follow 状态；
     - 通过 Web 层已有的 `switchMode()` 逻辑确保同一时刻只有一个“控制源”。

---

## 四、Web & HTTP 接口扩展规划

1. **新 HTTP 接口（规划名字）**
   - 例如：`/startNavPreset?id=0..5`：
     - 根据 id 选择 `g_nav_goals[id]`；
     - 调用高层 Mission FSM 启动对应任务；
   - 可选：`/getMissionStatus`：
     - 返回任务状态（IDLE / GOTO_PRE / ALIGN / IMPACT / RETURN / DONE / ERROR），以及当前子阶段进度信息。

2. **前端 Web 页面调整（web.h）**
   - 在现有导航卡片下方增加 6 个按钮：Goal1–Goal6（文案后续你定）：
     - 点击即 `fetch('/startNavPreset?id=X')`；
   - 可选增加任务状态显示区域（从 `/getMissionStatus` 拉取）。

---

## 五、具体实施步骤（后续修改代码时按此执行）

1. **Step 0：等待你提供 6 个目标点的实测坐标 & 期望姿态**
   - 形式：物理英寸坐标 + 大致朝向角度（度），以及 ToF 期望距离区间的大致想法。

2. **Step 1：新增 `nav_config.h` 并填入初始参数**
   - 把导航相关常量集中到该文件；
   - 定义 `nav_goal_t` 结构和 `g_nav_goals[6]` 默认值（可根据你给出的坐标初次填写）。

3. **Step 2：改造 `vive_navigation` 以使用 `nav_config.h`（不破坏现有 API）**
   - 将原本散落在 `vive_navigation.h` 里的宏改为从 `nav_config.h` 引用；
   - 保持 `vive_nav_*` 函数签名不变。

4. **Step 3：实现高层 Mission 状态机**
   - 新增状态枚举与内部数据结构；
   - 编写：
     - GOTO_PRE_POINT：调用现有 Vive 导航直到 `NAV_STATE_ARRIVED`；
     - ALIGN_TOF：ToF 慢速旋转对齐；
     - FORWARD_IMPACT / RETURN：直线前进/后退逻辑；
   - 确保出现 ToF 异常或超时时能安全停下并进入 ERROR。

5. **Step 4：扩展 `html510.cpp` HTTP 接口**
   - 实现 `/startNavPreset`、`/getMissionStatus` handler；
   - 根据当前任务状态控制底盘与导航模块的启停，保持与 Wall-follow 和手动控制的互斥。

6. **Step 5：更新 `web.h` 页面**
   - 添加 6 个预设目标按钮和任务状态显示；
   - 复用现有 `switchMode()` 逻辑，保证进入“导航模式”时自动停止墙循迹/手动控制。

7. **Step 6：上板调试与参数微调**
   - 在线打印 Mission 状态和 ToF 读数；
   - 逐一调整 `nav_config.h` 里的 ToF 区间、前进距离和转速，使撞击行为稳定可靠。

> 以上内容仅为规划文档，下一步在你确认无误并提供 6 个坐标后，再按上述步骤逐项修改代码。
