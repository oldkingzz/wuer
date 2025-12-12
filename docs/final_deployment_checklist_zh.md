# 最终部署检查清单 (Final Deployment Checklist)

## 1. 系统状态 (System Status)
- **沿墙走 (Wall Following V2)**: 已完全实现 (8个阶段)。
  - **逻辑**: RT(起点) -> Top(上半圆) -> LT(左上角) -> Long(左长边/盲区) -> LB(左下角) -> Bottom(下半圆) -> RB(右下角) -> Long(右长边) -> Finish(终点停车)。
  - **验证**: 语法修复完成，Start阶段增加了对齐(Alignment)逻辑。

- **导航系统 (Navigation System)**: 已实现 `vive_navigation.cpp`。
  - **定位**: EKF (编码器 + Vive) 融合。
  - **配置**: 标定点和目标点已预留。
  - **运动**: 纯追踪 (Pure Pursuit) + A* 路径规划。

- **底盘控制 (Chassis Control)**: PID 参数已调整。
  - **PID**: 右边电机针对漏液问题进行了补偿 (调高Kd，调低Ki)。
  - **运动学**: 差速模型已验证。

## 2. 明天实车测试步骤 (Pre-Deployment Steps)
1.  **标定 (Calibration)**: 
    *   测量场地 4 个角落 + 1 个中心点的 Vive 坐标。
    *   填入 `src/nav_config.cpp` 中的 `g_nav_calib_points` 数组。
2.  **Vive 检查**: 确保 Vive 传感器能看到基站，数据有效 (X, Y < 8192)。
3.  **电机测试**: 观察 PID 响应。如果右轮震荡，尝试减小 `PID_RIGHT_KP` 或增大 `PID_RIGHT_KD`。
4.  **沿墙距离**: 
    *   通用目标距离: **50mm**。
    *   左侧长边: **70mm**。
    *   如果发现太近或太远，在 `wall_following_v2.cpp` 中调整常量。

## 3. 逻辑流程验证 (Logic Flow Verification)
- **Start (起点)**: `process_convex_corner_rt` 会先直行停住，左转90度，然后进行 **8-12cm 距离对齐**，最后再转90度。这能确保进弯姿态正对墙壁。
- **Loop (绕圈)**: 逆时针绕行 (全程左转)，右侧沿墙。符合比赛标准。
- **Finish (终点)**: 在右侧长边 (`process_long_edge_right`) 遇到障碍物后会 **完全停车**。

## 常见问题 (FAQ)
- **Q: 寻墙和导航冲突吗？**
  - **A: 是的，冲突！** 请不要同时开启。它们都会控制轮子，同时开会“打架”。测试寻墙时，请确保导航已停止；测试导航时，请确保寻墙已停止。
- **Q: 按下网页按钮后，车会直接动吗？**
  - **A: 是的！** 
    - 点击 `Start Wall Follow` -> `process_convex_corner_rt` 立即执行 -> 车会马上直行。
    - 点击导航目标 -> 路径规划成功 -> 车会马上开始走。
    - **建议**: 先把车架起来按开始，确认轮子转动方向对了，再放在地上。

**祝测试顺利！(Ready for deployment)**
