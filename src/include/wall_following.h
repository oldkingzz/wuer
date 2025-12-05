/**
 * @file wall_following.h
 * @brief Wall Following Algorithm Module
 * 
 * 差速轮机器人寻墙算法模块
 * Wall Following Algorithm for Differential Drive Robot
 */

#ifndef WALL_FOLLOWING_H
#define WALL_FOLLOWING_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 / Configuration Parameters ========== */

/**
 * 机器人物理参数 / Robot Physical Parameters
 * 
 * 碰撞直径: 250mm (半径125mm)
 * Collision diameter: 250mm (radius 125mm)
 */
#define ROBOT_COLLISION_RADIUS_MM   125     // 机器人碰撞半径 (mm)

/**
 * ToF传感器布局 / ToF Sensor Layout
 * 
 * 侧面两个ToF之间的距离: 23mm
 * Distance between left and right ToF: 23mm
 */
#define TOF_SIDE_SPACING_MM         23      // 左右ToF之间的距离 (mm)

/**
 * 寻墙参数 / Wall Following Parameters
 * 
 * 基于碰撞半径125mm，目标距离设置为150mm (125mm + 25mm安全余量)
 * Based on collision radius 125mm, target distance set to 150mm (125mm + 25mm safety margin)
 */
#define WALL_DISTANCE_TARGET        150     // 目标循墙距离 (mm)
#define FRONT_OBSTACLE_THRESHOLD    180     // 前方障碍物阈值 (mm) - 需要预留转向空间
#define SIDE_WALL_LOST_THRESHOLD    450     // 侧面墙壁消失阈值 (mm) - 检测凸角
#define SIDE_WALL_FOUND_THRESHOLD   350     // 侧面墙壁检测阈值 (mm) - 重新找到墙

/**
 * 运动参数 / Motion Parameters
 */
#define WF_FORWARD_SPEED            0.15f   // 前进速度 (m/s)
#define WF_TURN_ANGULAR_SPEED       1.0f    // 转向角速度 (rad/s)
#define WF_CLEARANCE_DISTANCE       0.30f   // 绕过凸角后的延迟距离 (m)

/**
 * PID参数 / PID Parameters
 */
#define WF_PID_KP                   0.003f  // 比例增益
#define WF_PID_KI                   0.0f    // 积分增益
#define WF_PID_KD                   0.005f  // 微分增益

/**
 * IMU转向精度 / IMU Turn Accuracy
 */
#define WF_TURN_ANGLE_TOLERANCE     2.0f    // 转向角度容差 (度)

/* ========== 状态机定义 / State Machine Definition ========== */

/**
 * @brief 寻墙状态机
 * Wall Following State Machine
 */
typedef enum {
    WF_STATE_IDLE,          // 空闲状态
    WF_STATE_FIND_WALL,     // 寻找墙壁
    WF_STATE_FOLLOW_WALL,   // 循墙前进
    WF_STATE_TURN_AWAY,     // 遇到凹角，转离墙壁
    WF_STATE_TURN_TOWARD,   // 遇到凸角，转向墙壁
    WF_STATE_CLEARANCE,     // 绕过凸角后的延迟直行
    WF_STATE_STOPPED        // 停止
} wall_follow_state_t;

/**
 * @brief 寻墙状态信息
 * Wall Following Status Information
 */
typedef struct {
    wall_follow_state_t state;      // 当前状态
    uint16_t tof_front;             // 前方ToF读数 (mm)
    uint16_t tof_left;              // 左侧ToF读数 (mm)
    uint16_t tof_right;             // 右侧ToF读数 (mm)
    float current_heading;          // 当前朝向 (度)
    float target_heading;           // 目标朝向 (度)
    float total_distance;           // 累计行驶距离 (m)
    bool is_running;                // 是否正在运行
} wall_follow_status_t;

/* ========== API函数 / API Functions ========== */

/**
 * @brief 初始化寻墙系统
 * Initialize wall following system
 * 
 * 注意：必须在ToF、IMU、编码器、底盘初始化之后调用
 * Note: Must be called after ToF, IMU, encoder, and chassis initialization
 * 
 * @return ESP_OK 成功 / Success
 *         ESP_FAIL 失败 / Failure
 */
esp_err_t wall_following_init(void);

/**
 * @brief 启动寻墙
 * Start wall following
 * 
 * @return ESP_OK 成功 / Success
 *         ESP_FAIL 失败 / Failure
 */
esp_err_t wall_following_start(void);

/**
 * @brief 停止寻墙
 * Stop wall following
 * 
 * @return ESP_OK 成功 / Success
 *         ESP_FAIL 失败 / Failure
 */
esp_err_t wall_following_stop(void);

/**
 * @brief 获取寻墙状态
 * Get wall following status
 * 
 * @param status 状态信息指针 / Pointer to status structure
 * @return ESP_OK 成功 / Success
 *         ESP_FAIL 失败 / Failure
 */
esp_err_t wall_following_get_status(wall_follow_status_t *status);

/**
 * @brief 检查寻墙系统是否正在运行
 * Check if wall following is running
 * 
 * @return true 正在运行 / Running
 *         false 未运行 / Not running
 */
bool wall_following_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // WALL_FOLLOWING_H

