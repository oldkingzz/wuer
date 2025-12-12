/**
 * @file nav_config.h
 * @brief Navigation configuration: preset goals & Vive calibration samples
 *
 * 所有和导航目标点、Vive 实测坐标相关的配置都集中在这里，
 * 以后你只需要改这里的数值，不用到处翻代码。
 */

#ifndef NAV_CONFIG_H
#define NAV_CONFIG_H

#include "robot_config.h" // 提供 ROBOT_BASE_LINEAR_SPEED，方便统一调整速度
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Robot Configuration
 */
#define ROBOT_RADIUS_INCH 6.0f // 5.5" actual + 0.5" safety

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 预设导航目标点（在地图坐标系中）
 *
 * map_x / map_y 使用栅格地图的像素坐标（1 像素 = 1 英寸）。
 * heading_deg 是机器人在该点期望的车头朝向（单位：度，0~360）。
 */
typedef struct {
  int16_t map_x;     ///< Map X coordinate in pixels (field inches)
  int16_t map_y;     ///< Map Y coordinate in pixels (field inches)
  float heading_deg; ///< Desired robot heading at this goal (degrees)
} nav_goal_t;

/**
 * @brief 每个目标点对应的一组 Vive 原始坐标样本
 *
 * 因为有两个 Vive 传感器，所以在每个目标点上可以记录：
 * - vive1_x, vive1_y
 * - vive2_x, vive2_y
 *
 * 这些值是 Vive 原始坐标（0~8191），用于后续做更精确的
 * Vive->地图坐标标定（旋转 + 缩放 + 平移）。
 */
typedef struct {
  uint16_t vive1_x;
  uint16_t vive1_y;
  uint16_t vive2_x;
  uint16_t vive2_y;
} nav_goal_vive_sample_t;

/**
 * @brief 标定点结构体 (Calibration Point)
 * 用于通过 Affine Transform 计算 Vive -> Map 的映射矩阵
 */
typedef struct {
  float map_x;      ///< 地图上的物理坐标 X (inches)
  float map_y;      ///< 地图上的物理坐标 Y (inches)
  uint16_t vive1_x; ///< Vive1 原始坐标 X
  uint16_t vive1_y; ///< Vive1 原始坐标 Y
  uint16_t vive2_x; ///< Vive2 原始坐标 X
  uint16_t vive2_y; ///< Vive2 原始坐标 Y
} nav_calib_point_t;

/**
 * @brief 标定点数量 (建议 >= 4，覆盖四个角落)
 */
#define NAV_CALIB_POINT_COUNT 4

/**
 * @brief 预设目标点数量
 */
#define NAV_NUM_GOALS 6

/**
 * @brief 预设目标点的索引（后续可以按含义重命名）
 *
 * 目前只是占位，方便你后面根据实际含义改成：
 * 例如 LOW_TOWER_LEFT_PREP / HIGH_TOWER_RIGHT_IMPACT 等。
 */
typedef enum {
  NAV_GOAL_0 = 0,
  NAV_GOAL_1 = 1,
  NAV_GOAL_2 = 2,
  NAV_GOAL_3 = 3,
  NAV_GOAL_4 = 4,
  NAV_GOAL_5 = 5,
  NAV_GOAL_COUNT = NAV_NUM_GOALS
} nav_goal_id_t;

/**
 * @brief 低层 A* 导航控制参数（原本在 vive_navigation.h 里的 NAV_* 宏）
 *
 * 所有 A* 路径规划和路径跟随用到的常数都集中在这里，方便统一调整。
 * 单位保持和原代码一致。
 */

// 到达目标点的距离阈值 (像素)
#define NAV_ARRIVAL_THRESHOLD 10 // 10 pixels ≈ 5 inches

// 朝向对齐的角度阈值 (度)
#define NAV_HEADING_THRESHOLD 5.0f

// 最大线速度 (m/s) - 与 ROBOT_BASE_LINEAR_SPEED 保持一致
#define NAV_MAX_LINEAR_VELOCITY ROBOT_BASE_LINEAR_SPEED

// 最大角速度 (rad/s)
#define NAV_MAX_ANGULAR_VELOCITY 1.0f

// 路径跟踪前瞻距离 (像素)
#define NAV_LOOKAHEAD_DISTANCE 20 // 20 pixels ≈ 10 inches

// 路径重规划距离阈值 (像素)
#define NAV_REPLAN_THRESHOLD 50 // 50 pixels ≈ 25 inches

// 比例控制增益 - 距离控制
#define NAV_DISTANCE_KP 0.01f

// 比例控制增益 - 角度控制
#define NAV_HEADING_KP 0.02f

/**
 * @brief Mission 任务状态机相关参数（ToF 对齐 / 撞击 / 返回）
 *
 * 这些参数专门用于高层撞击任务，不影响基础 A* 导航逻辑。
 * 你以后只需要改这里，就能同时影响所有 Mission 行为。
 */

// ---------- GOTO_PRE_POINT：导航到预备点 ----------

// 导航到预备点的最大允许时间 (ms)
#define NAV_MISSION_GOTO_PREPOINT_TIMEOUT_MS 15000

// ---------- ALIGN_TOF：利用 ToF 原地慢速旋转对齐 ----------

// 期望前方 ToF 距离 (mm)
#define NAV_MISSION_ALIGN_FRONT_TARGET_MM 180
// 前方距离允许误差 (±mm)
#define NAV_MISSION_ALIGN_FRONT_TOL_MM 40

// 期望侧向（左前）ToF 距离 (mm)
#define NAV_MISSION_ALIGN_SIDE_TARGET_MM 150
// 侧向距离允许误差 (±mm)
#define NAV_MISSION_ALIGN_SIDE_TOL_MM 40

// 对齐判定所需的连续稳定时间 (ms)
#define NAV_MISSION_ALIGN_STABLE_TIME_MS 400

// ALIGN 阶段最长允许时间 (ms)
#define NAV_MISSION_ALIGN_MAX_TIME_MS 6000

// ALIGN 阶段原地旋转角速度 (rad/s)
#define NAV_MISSION_ALIGN_ANGULAR_SPEED 0.4f

// ---------- FORWARD_IMPACT：向前撞击 ----------

// 撞击阶段向前的线速度 (m/s)
#define NAV_MISSION_IMPACT_FORWARD_SPEED ROBOT_BASE_LINEAR_SPEED

// 撞击阶段最大前进距离 (mm)，超出则强制停止
#define NAV_MISSION_IMPACT_MAX_TRAVEL_MM 400

// 当前方 ToF 小于此距离 (mm) 时认为已经撞到/足够接近目标
#define NAV_MISSION_IMPACT_FRONT_STOP_MM 80

// 撞击阶段最大允许时间 (ms)
#define NAV_MISSION_IMPACT_MAX_TIME_MS 3000

// ---------- RETURN：后退回安全位置 ----------

// 后退时的线速度 (m/s)
#define NAV_MISSION_RETURN_SPEED (ROBOT_BASE_LINEAR_SPEED * 0.8f)

// 目标后退距离 (mm)，实际会与前进距离取一个较小值
#define NAV_MISSION_RETURN_TRAVEL_MM 300

// RETURN 阶段最大允许时间 (ms)
#define NAV_MISSION_RETURN_MAX_TIME_MS 3000

// ---------- 通用：Mission 中检查 ToF 数据新鲜度的超时时间 ----------

#define NAV_MISSION_TOF_FRESH_TIMEOUT_MS 200

/**
 * @brief 6 个预设导航目标点（地图坐标 + 期望朝向）
 */
extern nav_goal_t g_nav_goals[NAV_NUM_GOALS];

/**
 * @brief 对应 6 个目标点的 Vive 原始坐标样本（两个传感器各一对）
 */
extern nav_goal_vive_sample_t g_nav_goal_vive_samples[NAV_NUM_GOALS];

/**
 * @brief 标定点数据数组
 */
extern nav_calib_point_t g_nav_calib_points[NAV_CALIB_POINT_COUNT];

#ifdef __cplusplus
}
#endif

#endif // NAV_CONFIG_H
