/**
 * @file astar.cpp
 * @brief A* Path Planning Algorithm Implementation
 */

#include "include/astar.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "include/grid_map.h"
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ASTAR";

// A*节点结构
typedef struct astar_node {
  int16_t x, y;              // 位置
  float g_cost;              // 从起点到当前点的代价
  float h_cost;              // 从当前点到终点的启发式代价
  float f_cost;              // f = g + h
  struct astar_node *parent; // 父节点
  bool in_open_list;         // 是否在开放列表中
  bool in_closed_list;       // 是否在关闭列表中
} astar_node_t;

// 节点池（避免频繁malloc）- 使用动态分配
// 减小回 2500 以节省内存 (防止 Web Server 崩溃)
// 每个节点约32字节 -> 2500 * 32 = 80KB (加上 visited 8KB = 88KB)
// ESP32 Heap 紧张，不能开太大
#define NODE_POOL_SIZE 2500
static astar_node_t *g_node_pool = NULL;
static uint16_t g_node_pool_index = 0;

// 开放列表（简单数组实现，可优化为优先队列）- 使用动态分配
static astar_node_t **g_open_list = NULL;
static uint16_t g_open_list_size = 0;

// 访问标记数组 - 使用动态分配
static uint8_t (*g_visited)[MAP_WIDTH] = NULL;

static bool g_astar_initialized = false;

/**
 * @brief 重置节点池
 */
static void reset_node_pool(void) {
  g_node_pool_index = 0;
  g_open_list_size = 0;
  // 修复：g_visited是指针，不能用sizeof(g_visited)
  memset(g_visited, 0, MAP_HEIGHT * MAP_WIDTH * sizeof(uint8_t));
}

/**
 * @brief 从节点池分配节点
 */
static astar_node_t *allocate_node(void) {
  if (g_node_pool_index >= NODE_POOL_SIZE) {
    ESP_LOGE(TAG, "Node pool exhausted!");
    return NULL;
  }
  return &g_node_pool[g_node_pool_index++];
}

/**
 * @brief 计算启发式代价（欧几里得距离）
 */
static float heuristic(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
  float dx = (float)(x2 - x1);
  float dy = (float)(y2 - y1);
  return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 添加节点到开放列表
 */
static void add_to_open_list(astar_node_t *node) {
  if (g_open_list_size >= NODE_POOL_SIZE) {
    Serial.printf("ASTAR ERROR: Open list full! Size=%d\n", g_open_list_size);
    ESP_LOGE(TAG, "Open list full!");
    return;
  }
  g_open_list[g_open_list_size++] = node;
  node->in_open_list = true;
  // Serial.printf("  Added node (%d, %d) to OpenList. New Size=%d\n", node->x,
  //               node->y, g_open_list_size);
}

/**
 * @brief 从开放列表中找到f_cost最小的节点
 */
static astar_node_t *get_lowest_f_cost_node(void) {
  if (g_open_list_size == 0)
    return NULL;

  uint16_t lowest_index = 0;
  float lowest_f = g_open_list[0]->f_cost;

  for (uint16_t i = 1; i < g_open_list_size; i++) {
    if (g_open_list[i]->f_cost < lowest_f) {
      lowest_f = g_open_list[i]->f_cost;
      lowest_index = i;
    }
  }

  // 从开放列表中移除
  astar_node_t *node = g_open_list[lowest_index];
  g_open_list[lowest_index] = g_open_list[--g_open_list_size];
  node->in_open_list = false;

  return node;
}

/**
 * @brief 重建路径
 */
static void reconstruct_path(astar_node_t *goal_node, path_t *path) {
  // 从终点回溯到起点
  astar_node_t *current = goal_node;
  uint16_t count = 0;

  while (current != NULL && count < MAX_PATH_LENGTH) {
    count++;
    current = current->parent;
  }

  // 反向填充路径
  path->length = count;
  current = goal_node;

  for (int i = count - 1; i >= 0 && current != NULL; i--) {
    path->waypoints[i].x = current->x;
    path->waypoints[i].y = current->y;
    current = current->parent;
  }

  path->total_cost = goal_node->g_cost;
  path->valid = true;

  ESP_LOGI(TAG, "Path reconstructed: %d waypoints, cost=%.1f", path->length,
           path->total_cost);
}

/**
 * @brief 初始化A*算法
 */
esp_err_t astar_init(void) {
  if (g_astar_initialized) {
    ESP_LOGW(TAG, "A* already initialized");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing A* algorithm...");
  ESP_LOGI(TAG, "  Node pool size: %d", NODE_POOL_SIZE);
  ESP_LOGI(TAG, "  Max path length: %d", MAX_PATH_LENGTH);

  // 先检查可用内存
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  ESP_LOGI(TAG, "  Free heap before allocation: %u bytes", free_heap);
  ESP_LOGI(TAG, "  Largest free block: %u bytes", largest_block);

  // 先分配小的数组，再分配大的（减少碎片化）
  // 1. 动态分配访问标记数组 (8,640 bytes - 最小的)
  size_t visited_size = MAP_HEIGHT * MAP_WIDTH * sizeof(uint8_t);
  g_visited =
      (uint8_t(*)[MAP_WIDTH])heap_caps_malloc(visited_size, MALLOC_CAP_8BIT);
  if (g_visited == NULL) {
    ESP_LOGE(TAG, "Failed to allocate visited array (%u bytes)", visited_size);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "  Allocated visited array: %u bytes at %p", visited_size,
           g_visited);

  // 2. 动态分配开放列表 (约8KB - 中等)
  size_t open_list_size = NODE_POOL_SIZE * sizeof(astar_node_t *);
  g_open_list =
      (astar_node_t **)heap_caps_malloc(open_list_size, MALLOC_CAP_8BIT);
  if (g_open_list == NULL) {
    ESP_LOGE(TAG, "Failed to allocate open list (%u bytes)", open_list_size);
    free(g_visited);
    g_visited = NULL;
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "  Allocated open list: %u bytes at %p", open_list_size,
           g_open_list);

  // 3. 动态分配节点池 (约48KB - 最大的，最后分配)
  size_t node_pool_size = NODE_POOL_SIZE * sizeof(astar_node_t);
  g_node_pool =
      (astar_node_t *)heap_caps_malloc(node_pool_size, MALLOC_CAP_8BIT);
  if (g_node_pool == NULL) {
    ESP_LOGE(TAG, "Failed to allocate node pool (%u bytes)", node_pool_size);
    ESP_LOGE(TAG, "  Free heap: %u bytes",
             heap_caps_get_free_size(MALLOC_CAP_8BIT));
    ESP_LOGE(TAG, "  Largest block: %u bytes",
             heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    free(g_visited);
    free(g_open_list);
    g_visited = NULL;
    g_open_list = NULL;
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "  Allocated node pool: %u bytes at %p", node_pool_size,
           g_node_pool);

  size_t total_size = node_pool_size + open_list_size + visited_size;
  g_astar_initialized = true;
  ESP_LOGI(TAG, "A* initialized successfully (total: %u bytes, ~%u KB)",
           total_size, total_size / 1024);
  return ESP_OK;
}

/**
 * @brief A*路径规划主函数
 */
esp_err_t astar_plan_path(int16_t start_x, int16_t start_y, int16_t goal_x,
                          int16_t goal_y, path_t *path) {
  if (!g_astar_initialized || path == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Planning path from (%d,%d) to (%d,%d)", start_x, start_y,
           goal_x, goal_y);
  Serial.printf("ASTAR: Plan from (%d,%d) to (%d,%d)\n", start_x, start_y,
                goal_x, goal_y);

  // 检查起点和终点是否有效
  if (!grid_map_is_free(start_x, start_y)) {
    Serial.printf("ASTAR ERROR: Start blocked! Cost=%d\n",
                  grid_map_get_cost(start_x, start_y));
    ESP_LOGE(TAG, "Start position is blocked!");
    return ESP_FAIL;
  }
  if (!grid_map_is_free(goal_x, goal_y)) {
    Serial.printf("ASTAR ERROR: Goal blocked! Cost=%d\n",
                  grid_map_get_cost(goal_x, goal_y));
    ESP_LOGE(TAG, "Goal position is blocked!");
    return ESP_FAIL;
  }

  // 重置节点池和列表
  reset_node_pool();

  // 创建起点节点
  astar_node_t *start_node = allocate_node();
  if (start_node == NULL) {
    Serial.printf("ASTAR ERROR: Failed to allocate start node!\n");
    return ESP_FAIL;
  }

  start_node->x = start_x;
  start_node->y = start_y;
  start_node->g_cost = 0.0f;
  start_node->h_cost = heuristic(start_x, start_y, goal_x, goal_y);
  start_node->f_cost = start_node->h_cost;
  start_node->parent = NULL;
  start_node->in_open_list = false;
  start_node->in_closed_list = false;

  add_to_open_list(start_node);

  // A*主循环
  uint32_t iterations = 0;
  uint32_t last_log_iteration = 0;

  while (g_open_list_size > 0 && iterations < MAX_ASTAR_ITERATIONS) {
    iterations++;

    // 3. Debug Print (Every 1000 iter)
    if (iterations - last_log_iteration >= 1000) {
      ESP_LOGI(TAG, "A* iter %lu, open=%d, pool=%d", iterations,
               g_open_list_size, g_node_pool_index);
      last_log_iteration = iterations;
    }

    // 获取f_cost最小的节点
    astar_node_t *current = get_lowest_f_cost_node();
    if (current == NULL)
      break;

    // Skip if already processed (matches Python behavior)
    if (g_visited[current->y][current->x] == 2) {
      continue;
    }

    // 标记为已访问 (CLOSED)
    current->in_closed_list = true;
    g_visited[current->y][current->x] = 2; // 2 = CLOSED

    // 检查是否到达终点
    if (current->x == goal_x && current->y == goal_y) {
      // Serial.printf("Path found! Iterations: %lu\n", iterations);
      reconstruct_path(current, path);
      return ESP_OK;
    }

    // 扩展邻居节点（8方向）
    static const int16_t dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int16_t dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    static const float move_cost[] = {1.414f, 1.0f,   1.414f, 1.0f,
                                      1.0f,   1.414f, 1.0f,   1.414f};

    // Serial.printf("ASTAR DEBUG: MAP_DIM=(%d, %d)\n", MAP_WIDTH, MAP_HEIGHT);

    for (int i = 0; i < 8; i++) {
      int16_t nx = current->x + dx[i];
      int16_t ny = current->y + dy[i];
      // Serial.printf("  Check (%d, %d)...\n", nx, ny);

      // 边界检查
      if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) {
        continue;
      }

      // 障碍物检查
      if (!grid_map_is_free(nx, ny)) {
        // Serial.printf("  Skip (%d,%d): Blocked cost=%d\n", nx, ny,
        // grid_map_get_cost(nx, ny));
        continue;
      }

      // 状态检查:
      // 0 = NONE
      // 1 = OPEN (Pending)
      // 2 = CLOSED (Processed)
      // 如果已经在 Open 或 Closed，简单跳过以节省内存 (Suboptimal but safe)
      if (g_visited[ny][nx] > 0) {
        // Serial.printf("  Skip (%d,%d): Visited\n", nx, ny);
        continue;
      }

      // 计算新的g_cost
      uint8_t terrain_cost = grid_map_get_cost(nx, ny);
      float new_g_cost = current->g_cost + move_cost[i] * terrain_cost;

      // 创建或更新邻居节点
      astar_node_t *neighbor = allocate_node();
      if (neighbor == NULL) {
        // Serial.printf("ASTAR ERROR: Node pool exhausted at iter %lu\n",
        // iterations);
        ESP_LOGW(TAG, "Node pool exhausted at iteration %lu", iterations);
        break;
      }

      neighbor->x = nx;
      neighbor->y = ny;
      neighbor->g_cost = new_g_cost;
      neighbor->h_cost = heuristic(nx, ny, goal_x, goal_y);
      neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
      neighbor->parent = current;
      neighbor->in_open_list = false;
      neighbor->in_closed_list = false;

      add_to_open_list(neighbor);
      g_visited[ny][nx] = 1; // Mark as OPEN
    }
  }

  ESP_LOGW(TAG, "No path found after %lu iterations", iterations);
  Serial.printf("ASTAR ERROR: No path found after %lu iters\n", iterations);
  path->valid = false;
  path->length = 0;
  return ESP_FAIL;
}

/**
 * @brief 简化路径（移除冗余点）
 */
esp_err_t astar_simplify_path(path_t *path) {
  if (path == NULL || !path->valid || path->length < 3) {
    return ESP_OK; // 路径太短，无需简化
  }

  map_point_t simplified[MAX_PATH_LENGTH];
  uint16_t simplified_count = 0;

  // 始终保留起点
  simplified[simplified_count++] = path->waypoints[0];

  uint16_t i = 0;
  while (i < path->length - 1) {
    uint16_t j = path->length - 1;

    // 从当前点向终点方向寻找最远的可见点
    while (j > i + 1) {
      // 检查从i到j的直线是否可通行
      bool line_clear = true;

      int16_t x0 = path->waypoints[i].x;
      int16_t y0 = path->waypoints[i].y;
      int16_t x1 = path->waypoints[j].x;
      int16_t y1 = path->waypoints[j].y;

      // Bresenham直线算法检查
      int16_t dx = abs(x1 - x0);
      int16_t dy = abs(y1 - y0);
      int16_t sx = (x0 < x1) ? 1 : -1;
      int16_t sy = (y0 < y1) ? 1 : -1;
      int16_t err = dx - dy;

      int16_t x = x0, y = y0;

      while (true) {
        if (!grid_map_is_free(x, y)) {
          line_clear = false;
          break;
        }

        if (x == x1 && y == y1)
          break;

        int16_t e2 = 2 * err;
        if (e2 > -dy) {
          err -= dy;
          x += sx;
        }
        if (e2 < dx) {
          err += dx;
          y += sy;
        }
      }

      if (line_clear) {
        break; // 找到最远可见点
      }

      j--;
    }

    // 添加可见点
    simplified[simplified_count++] = path->waypoints[j];
    i = j;

    if (simplified_count >= MAX_PATH_LENGTH) {
      ESP_LOGW(TAG, "Simplified path buffer full");
      break;
    }
  }

  // 更新路径
  memcpy(path->waypoints, simplified, simplified_count * sizeof(map_point_t));
  path->length = simplified_count;

  ESP_LOGI(TAG, "Path simplified to %d waypoints", simplified_count);
  return ESP_OK;
}

/**
 * @brief 获取路径上的下一个目标点
 */
esp_err_t astar_get_next_target(const path_t *path, int16_t current_x,
                                int16_t current_y, float lookahead_distance,
                                int16_t *target_x, int16_t *target_y) {
  if (path == NULL || !path->valid || path->length == 0) {
    return ESP_FAIL;
  }

  // 找到距离当前位置最近的路径点
  uint16_t closest_index = 0;
  float min_dist = 1e9f;

  for (uint16_t i = 0; i < path->length; i++) {
    float dx = path->waypoints[i].x - current_x;
    float dy = path->waypoints[i].y - current_y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      closest_index = i;
    }
  }

  // 从最近点开始，找到前瞻距离处的点
  for (uint16_t i = closest_index; i < path->length; i++) {
    float dx = path->waypoints[i].x - current_x;
    float dy = path->waypoints[i].y - current_y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist >= lookahead_distance) {
      *target_x = path->waypoints[i].x;
      *target_y = path->waypoints[i].y;
      return ESP_OK;
    }
  }

  // 如果没有找到，返回终点
  *target_x = path->waypoints[path->length - 1].x;
  *target_y = path->waypoints[path->length - 1].y;
  return ESP_OK;
}

/**
 * @brief 检查路径是否仍然有效
 */
bool astar_is_path_valid(const path_t *path) {
  if (path == NULL || !path->valid) {
    return false;
  }

  // 检查路径上的每个点是否仍然可通行
  for (uint16_t i = 0; i < path->length; i++) {
    if (!grid_map_is_free(path->waypoints[i].x, path->waypoints[i].y)) {
      return false;
    }
  }

  return true;
}

/**
 * @brief 清理A*算法，释放内存
 */
void astar_deinit(void) {
  if (g_node_pool != NULL) {
    free(g_node_pool);
    g_node_pool = NULL;
  }

  if (g_open_list != NULL) {
    free(g_open_list);
    g_open_list = NULL;
  }

  if (g_visited != NULL) {
    free(g_visited);
    g_visited = NULL;
  }

  g_astar_initialized = false;
  ESP_LOGI(TAG, "A* memory freed");
}
