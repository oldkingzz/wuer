/**
 * @file astar.h
 * @brief A* Path Planning Algorithm
 *
 * A*路径规划算法 - 用于在栅格地图上寻找最优路径
 * A* path planning algorithm for finding optimal path on grid map
 */

#ifndef ASTAR_H
#define ASTAR_H

#include "esp_err.h"
#include "grid_map.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置参数 / Configuration Parameters ========== */

/**
 * 最大路径点数量
 * Maximum number of waypoints in path
 */
#define MAX_PATH_LENGTH 500

/**
 * A*搜索最大迭代次数
 * Maximum iterations for A* search
 */
#define MAX_ASTAR_ITERATIONS 10000

/* ========== 数据结构 / Data Structures ========== */

/**
 * @brief 路径结构
 * Path structure containing waypoints
 */
typedef struct {
  map_point_t waypoints[MAX_PATH_LENGTH]; // 路径点数组
  uint16_t length;                        // 路径点数量
  float total_cost;                       // 总代价
  bool valid;                             // 路径是否有效
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
esp_err_t astar_plan_path(int16_t start_x, int16_t start_y, int16_t goal_x,
                          int16_t goal_y, path_t *path);

/**
 * @brief 简化路径（移除冗余点）
 * Simplify path by removing redundant waypoints
 *
 * @param path 输入/输出路径
 * @return ESP_OK on success
 */
esp_err_t astar_simplify_path(path_t *path);

/**
 * @brief 获取路径上的下一个目标点 (Get next target point on path)
 *
 * @param path 路径指针
 * @param current_x 当前X坐标
 * @param current_y 当前Y坐标
 * @param lookahead_distance 前瞻距离 (pixels)
 * @param start_index 搜索起始索引 (防止回退) / Start search index to prevent
 * backtracking
 * @param target_x 输出目标X
 * @param target_y 输出目标Y
 * @param out_index 输出找到的目标点索引 / Output index of the found target
 * @return ESP_OK 成功, ESP_FAIL 失败
 */
esp_err_t astar_get_next_target(const path_t *path, int16_t current_x,
                                int16_t current_y, float lookahead_distance,
                                uint16_t start_index, int16_t *target_x,
                                int16_t *target_y, uint16_t *out_index);

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
