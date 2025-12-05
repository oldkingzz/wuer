/**
 * @file astar.h
 * @brief A* Path Planning Algorithm
 * 
 * A*路径规划算法 - 用于在栅格地图上寻找最优路径
 * A* path planning algorithm for finding optimal path on grid map
 */

#ifndef ASTAR_H
#define ASTAR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "grid_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 / Configuration Parameters ========== */

/**
 * 最大路径点数量
 * Maximum number of waypoints in path
 */
#define MAX_PATH_LENGTH         500

/**
 * A*搜索最大迭代次数
 * Maximum iterations for A* search
 */
#define MAX_ASTAR_ITERATIONS    10000

/* ========== 数据结构 / Data Structures ========== */

/**
 * @brief 路径结构
 * Path structure containing waypoints
 */
typedef struct {
    map_point_t waypoints[MAX_PATH_LENGTH];  // 路径点数组
    uint16_t length;                         // 路径点数量
    float total_cost;                        // 总代价
    bool valid;                              // 路径是否有效
} path_t;

/* ========== API函数 / API Functions ========== */

/**
 * @brief 初始化A*算法
 * Initialize A* algorithm
 * 
 * @return ESP_OK on success
 */
esp_err_t astar_init(void);

/**
 * @brief 使用A*算法规划路径
 * Plan path using A* algorithm
 * 
 * @param start_x 起点X坐标 (像素)
 * @param start_y 起点Y坐标 (像素)
 * @param goal_x 终点X坐标 (像素)
 * @param goal_y 终点Y坐标 (像素)
 * @param path 输出路径
 * @return ESP_OK if path found, ESP_FAIL if no path exists
 */
esp_err_t astar_plan_path(int16_t start_x, int16_t start_y, 
                          int16_t goal_x, int16_t goal_y, 
                          path_t *path);

/**
 * @brief 简化路径（移除冗余点）
 * Simplify path by removing redundant waypoints
 * 
 * @param path 输入/输出路径
 * @return ESP_OK on success
 */
esp_err_t astar_simplify_path(path_t *path);

/**
 * @brief 获取路径上的下一个目标点
 * Get next target waypoint on path
 * 
 * @param path 路径
 * @param current_x 当前X坐标 (像素)
 * @param current_y 当前Y坐标 (像素)
 * @param lookahead_distance 前瞻距离 (像素)
 * @param target_x 输出目标X坐标
 * @param target_y 输出目标Y坐标
 * @return ESP_OK if target found, ESP_FAIL if reached end
 */
esp_err_t astar_get_next_target(const path_t *path, 
                                int16_t current_x, int16_t current_y,
                                float lookahead_distance,
                                int16_t *target_x, int16_t *target_y);

/**
 * @brief 检查路径是否仍然有效（无新障碍物）
 * Check if path is still valid (no new obstacles)
 * 
 * @param path 路径
 * @return true if valid, false if blocked
 */
bool astar_is_path_valid(const path_t *path);

/**
 * @brief 清理A*算法，释放内存
 * Deinitialize A* and free memory
 */
void astar_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // ASTAR_H

