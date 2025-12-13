/**
 * @file nav_config.cpp
 * @brief Default values for navigation goals & Vive calibration samples
 *
 * 所有数值目前都是占位值，仅用于结构搭建。
 * 之后你在场地上量完 6 个点的实际坐标后，可以直接改这里：
 * - g_nav_goals[]      ：改成真实的地图坐标和期望朝向
 * - g_nav_goal_vive_samples[] ：改成对应位置的两路 Vive 原始坐标
 */

#include "include/nav_config.h"

// 注意：地图像素坐标基于现有 grid_map 约定：
// - X 方向范围大约是 [0, 144] 英寸
// - Y 方向范围大约是 [0, 60]  英寸
// 这里先随便给出 6 个分布在场地不同区域的占位点，方便你直观调试。

nav_goal_t g_nav_goals[NAV_NUM_GOALS] = {
    // NAV_GOAL_0: 测试点 (40, 100)，车头朝 +X 方向
    {40, 100, 0.0f},
    // NAV_GOAL_1: high tower_red，车头朝 +X 方向
    {6, 82, 180.0f},
    // NAV_GOAL_2: low tower_red，车头朝 -Y 方向
    {35, 80, 270.0f},
    // NAV_GOAL_3: low tower_blue，车头朝 +Y 方向
    {35, 60, 90.0f},
    // NAV_GOAL_4: red nexus，车头朝 +Y 方向
    {35, 128, 90.0f},
    // NAV_GOAL_5: blue nexus，车头朝 -Y 方向
    {35, 13, 270.0f},
};

// Vive 原始坐标的占位值：
// 这里只是保证数值落在 [1000, 8000] 的合理范围内，
// 真实标定时，你在每个目标点停稳后读出两路 Vive(x,y)，
// 把对应的值直接改写到下面这张表里即可。

nav_goal_vive_sample_t g_nav_goal_vive_samples[NAV_NUM_GOALS] = {
    // NAV_GOAL_0 对应的 Vive 坐标样本（占位）
    {2717, 3676, 2723, 3401},
    // NAV_GOAL_1
    {2732, 4903, 2733, 4629},
    // NAV_GOAL_2
    {4476, 4691, 4688, 4620},
    // NAV_GOAL_3
    {4713, 3691, 4468, 3693},
    // NAV_GOAL_4
    {4732, 6410, 4510, 4493},
    // NAV_GOAL_5
    {4410, 1909, 4633, 1897},
};

// ==========================================
// 标定点数据（6 点法）
// ==========================================
// 说明：
// 1. 所有测量点机器人必须朝向 +X 方向（一致朝向！）
// 2. 格式：{地图X(英寸), 地图Y(英寸), Vive1_X, Vive1_Y, Vive2_X, Vive2_Y}
// 3. 建议在场地中心区域取点，因为 Vive 在该区域最准
// 4. 避开坡道区域 (Y: 16-128)
// 5. 6 个点能更好地补偿高度导致的透视畸变

nav_calib_point_t g_nav_calib_points[NAV_CALIB_POINT_COUNT] = {
    // ====== 请在下方填入你的标定数据 (6点法 - 中心区域聚集) ======
    // 既然角落信号不好，我们在 (40, 72) 附近的稳定区域取点。
    // 这里采用一个包围 (40, 72) 的局部网格，确保数据质量。

    // Point 1: Center-Left-Low
    {30.0f, 72.0f, 4268, 4289, 4031, 3995},

    // Point 2: Center-Mid-Low
    {40.0f, 62.0f, 4789, 3503, 4522, 3501},

    // Point 3: Center-Right-Low
    {51.0f, 72.0f, 5297, 4008, 5060, 3994},

    // Point 4: Center-Left-High
    {40.0f, 83.0f, 4778, 4592, 4534, 4592},

    // Point 5: Center-Mid-High (Center Point)
    {49.0f, 90.0f, 5174, 4946, 4953, 4943},

    // Point 6: Center-Right-High
    {29.0f, 54.0f, 4242, 3157, 3984, 3147}};
