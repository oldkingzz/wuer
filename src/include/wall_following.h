/**
 * @file wall_following.h
 * @brief 寻墙算法 - 沿地图边缘行驶
 * 
 * 起始位置：右上角（北侧Nexus右侧）
 * 路径：沿地图边缘逆时针行驶一圈
 */

#ifndef WALL_FOLLOWING_H
#define WALL_FOLLOWING_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 状态机定义 ========== */

typedef enum {
    WF_STATE_IDLE,          // 空闲
    WF_STATE_INIT_POS,      // 计算初始位置
    WF_STATE_FOLLOW_WALL,   // 循墙前进
    WF_STATE_STOPPED        // 停止
} wall_follow_state_t;

/* ========== 状态信息 ========== */

typedef struct {
    wall_follow_state_t state;      // 当前状态
    uint16_t tof_front;             // 前方ToF (mm)
    uint16_t tof_left;              // 左侧ToF (mm)
    uint16_t tof_right;             // 右侧ToF (mm)
    float current_heading;          // 当前朝向 (度)
    float target_heading;           // 目标朝向 (度)
    float total_distance;           // 累计距离 (m)
    bool is_running;                // 是否运行
} wall_follow_status_t;

/* ========== API函数 ========== */

/**
 * @brief 初始化寻墙系统
 */
esp_err_t wall_following_init(void);

/**
 * @brief 启动寻墙
 */
esp_err_t wall_following_start(void);

/**
 * @brief 停止寻墙
 */
esp_err_t wall_following_stop(void);

/**
 * @brief 获取状态
 */
esp_err_t wall_following_get_status(wall_follow_status_t *status);

/**
 * @brief 检查是否运行
 */
bool wall_following_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // WALL_FOLLOWING_H

