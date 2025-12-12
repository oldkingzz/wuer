/**
 * @file grid_map.cpp
 * @brief MEAM 5100 ROBA Field Grid Map Implementation
 *
 * 坐标系定义 (Coordinate System):
 * - 原点 (0,0): 场地左下角
 * - X轴: 宽度方向 0-60"
 * - Y轴: 长度方向 0-144"
 * - 地图存储: g_grid_map[y][x]
 */

#include "include/grid_map.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

static const char *TAG = "GRID_MAP";

// 全局地图数组指针
// 注意: 实际地图尺寸是 60(宽) x 144(长)
// 但是 MAP_WIDTH=144, MAP_HEIGHT=60 (为了兼容头文件定义)
// 所以我们需要转置坐标: 物理(x,y) -> 数组[x][y]
static uint8_t (*g_grid_map)[MAP_WIDTH] = NULL;
static bool g_map_initialized = false;

// Calibration parameters for Affine Transform
// X_map = a*X_v + b*Y_v + c
// Y_map = d*X_v + e*Y_v + f
static float g_calib_a = -0.007324f; // Default (approx from 60/8192)
static float g_calib_b = 0.0f;
static float g_calib_c = 60.0f; // Offset (approx)
static float g_calib_d = 0.0f;
static float g_calib_e = -0.017578f; // Default (approx from 144/8192)
static float g_calib_f = 144.0f;     // Offset (approx)

static bool g_is_calibrated = false;

// Helper: Inverse 3x3 Matrix
static bool invert3x3(float m[3][3], float inv[3][3]) {
  float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
              m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
              m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  if (fabs(det) < 1e-6)
    return false; // Singular matrix

  float invDet = 1.0f / det;

  inv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invDet;
  inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
  inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

  inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
  inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
  inv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invDet;

  inv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invDet;
  inv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invDet;
  inv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invDet;

  return true;
}

/**
 * @brief Solve Least Squares for Affine Parameters
 * Solves X_map = a*X_v + b*Y_v + c
 * Form: [Xv Yv 1] * [a; b; c] = [Xmap]
 * Normal Eq: (A^T A) * beta = A^T Y
 */
static bool solve_affine_parameters(const nav_calib_point_t *points, int count,
                                    float *p_a, float *p_b, float *p_c,
                                    float *p_d, float *p_e, float *p_f) {
  if (count < 3)
    return false;

  // Construct A^T * A (3x3 symmetric) and A^T * ImageX/Y (3x1)
  float AtA[3][3] = {0};
  float AtX[3] = {0}; // For solving a,b,c (Target: Map X)
  float AtY[3] = {0}; // For solving d,e,f (Target: Map Y)

  for (int i = 0; i < count; i++) {
    // 取两个传感器的中心点进行标定 / Average the two sensors
    float x = ((float)points[i].vive1_x + (float)points[i].vive2_x) / 2.0f;
    float y = ((float)points[i].vive1_y + (float)points[i].vive2_y) / 2.0f;
    float mx = points[i].map_x;
    float my = points[i].map_y;

    // AtA
    AtA[0][0] += x * x;
    AtA[0][1] += x * y;
    AtA[0][2] += x;

    AtA[1][0] += x * y; // Symmetric
    AtA[1][1] += y * y;
    AtA[1][2] += y;

    AtA[2][0] += x; // Symmetric
    AtA[2][1] += y;
    AtA[2][2] += 1.0f;

    // AtX (Target Map X)
    AtX[0] += x * mx;
    AtX[1] += y * mx;
    AtX[2] += mx;

    // AtY (Target Map Y)
    AtY[0] += x * my;
    AtY[1] += y * my;
    AtY[2] += my;
  }

  float invAtA[3][3];
  if (!invert3x3(AtA, invAtA)) {
    ESP_LOGE(TAG, "Calibration data is degenerate (singular matrix)");
    return false;
  }

  // Solve for a, b, c
  *p_a = invAtA[0][0] * AtX[0] + invAtA[0][1] * AtX[1] + invAtA[0][2] * AtX[2];
  *p_b = invAtA[1][0] * AtX[0] + invAtA[1][1] * AtX[1] + invAtA[1][2] * AtX[2];
  *p_c = invAtA[2][0] * AtX[0] + invAtA[2][1] * AtX[1] + invAtA[2][2] * AtX[2];

  // Solve for d, e, f
  *p_d = invAtA[0][0] * AtY[0] + invAtA[0][1] * AtY[1] + invAtA[0][2] * AtY[2];
  *p_e = invAtA[1][0] * AtY[0] + invAtA[1][1] * AtY[1] + invAtA[1][2] * AtY[2];
  *p_f = invAtA[2][0] * AtY[0] + invAtA[2][1] * AtY[1] + invAtA[2][2] * AtY[2];

  return true;
}

esp_err_t grid_map_calibrate_affine(const nav_calib_point_t *points,
                                    int count) {
  float a, b, c, d, e, f;

  ESP_LOGI(TAG, "Starting Affine Calibration with %d points...", count);

  if (solve_affine_parameters(points, count, &a, &b, &c, &d, &e, &f)) {
    g_calib_a = a;
    g_calib_b = b;
    g_calib_c = c;
    g_calib_d = d;
    g_calib_e = e;
    g_calib_f = f;
    g_is_calibrated = true;

    ESP_LOGI(TAG, "Calibration SUCCESS:");
    ESP_LOGI(TAG, "  MapX = %.4f*Vx + %.4f*Vy + %.2f", a, b, c);
    ESP_LOGI(TAG, "  MapY = %.4f*Vx + %.4f*Vy + %.2f", d, e, f);
    return ESP_OK;
  } else {
    ESP_LOGE(TAG, "Calibration FAILED");
    return ESP_FAIL;
  }
}

/**
 * @brief 辅助函数：添加障碍物（可选膨胀）
 * @param x_in 物理X坐标 (0-60")
 * @param y_in 物理Y坐标 (0-144")
 * @param w_in 宽度 (X方向)
 * @param h_in 高度 (Y方向)
 * @param cost 代价
 * @param inflate_radius 膨胀半径 (inches)
 */
static void add_obstacle_inch(float x_in, float y_in, float w_in, float h_in,
                              uint8_t cost, float inflate_radius = 0.0f) {
  // 膨胀矩形 / Inflate rectangle
  float x_start_in = x_in - inflate_radius;
  float y_start_in = y_in - inflate_radius;
  float w_total_in = w_in + 2 * inflate_radius;
  float h_total_in = h_in + 2 * inflate_radius;

  int16_t x_start = (int16_t)(x_start_in * MAP_RESOLUTION);
  int16_t y_start = (int16_t)(y_start_in * MAP_RESOLUTION);
  int16_t w_pixel = (int16_t)(w_total_in * MAP_RESOLUTION);
  int16_t h_pixel = (int16_t)(h_total_in * MAP_RESOLUTION);

  // 物理坐标: X(0-60), Y(0-144)
  // 数组索引: g_grid_map[x][y] 因为我们转置了
  for (int16_t x = x_start; x < x_start + w_pixel; x++) {
    for (int16_t y = y_start; y < y_start + h_pixel; y++) {
      if (x >= 0 && x < 60 && y >= 0 && y < 144) {
        // 如果是膨胀区域，只覆盖 Cost更低 的区域 (避免覆盖已有的 Higher Cost)
        // 但这里简化：直接覆盖，如果是 WALL/OBSTACLE 都是 255
        if (g_grid_map[y][x] < cost) {
          g_grid_map[y][x] = cost;
        }
      }
    }
  }
}

/**
 * @brief 初始化栅格地图
 *
 * 新地图布局 (ROBA 2025):
 * - 场地: 60"(宽X) x 144"(长Y)
 * - 高地区域: X:[0,19], 包含坡道和平台
 * - 低地区域: X:[19,60]
 * - 基地: 南(30, 2.5), 北(30, 141.5)
 * - 塔: 高地塔(9.5, 72), 低地塔(40, 72)
 */
esp_err_t grid_map_init(void) {
  if (g_map_initialized) {
    ESP_LOGW(TAG, "Grid map already initialized");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing ROBA 2025 grid map...");
  ESP_LOGI(TAG, "  Physical field: 60\"(W) x 144\"(L)");
  ESP_LOGI(TAG, "  Map array size: %d x %d pixels", MAP_WIDTH, MAP_HEIGHT);
  ESP_LOGI(TAG, "  Resolution: %d pixels/inch", MAP_RESOLUTION);

  // 检查可用内存
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  ESP_LOGI(TAG, "  Free heap: %u bytes, Largest block: %u bytes", free_heap,
           largest_block);

  // 动态分配地图内存: 60 x 144 = 8,640 bytes
  size_t map_size = 60 * 144 * sizeof(uint8_t);
  g_grid_map =
      (uint8_t(*)[MAP_WIDTH])heap_caps_malloc(map_size, MALLOC_CAP_8BIT);

  if (g_grid_map == NULL) {
    ESP_LOGE(TAG, "Failed to allocate %u bytes for grid map", map_size);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "  Allocated %u bytes at %p", map_size, g_grid_map);

  // 初始化为平地
  memset(g_grid_map, COST_FREE, map_size);

  // ========== 1. 场地边界墙壁 (Boundary Walls) ==========
  ESP_LOGI(TAG, "  Drawing boundary walls (Inflated by %.1f\")...",
           ROBOT_RADIUS_INCH);

  // 使用 add_obstacle_inch 并带有膨胀，自动处理边界检查
  // 墙壁本身厚度设为 0.1"，加上 Robot Radius
  // 左墙 (X=0)
  add_obstacle_inch(0.0f, 0.0f, 0.0f, 144.0f, COST_OBSTACLE, ROBOT_RADIUS_INCH);
  // 右墙 (X=60) -> X=55.0 (Constraint: Force Center Path)
  add_obstacle_inch(55.0f, 0.0f, 0.0f, 144.0f, COST_OBSTACLE,
                    ROBOT_RADIUS_INCH);
  // 南墙 (Y=0)
  add_obstacle_inch(0.0f, 0.0f, 60.0f, 0.0f, COST_OBSTACLE, ROBOT_RADIUS_INCH);
  // 北墙 (Y=144)
  add_obstacle_inch(0.0f, 144.0f, 60.0f, 0.0f, COST_OBSTACLE,
                    ROBOT_RADIUS_INCH);

  // ========== 2. 高地结构 (X: 0-19) ==========
  ESP_LOGI(TAG, "  Drawing high level structure & Wall...");

  // 高地边缘墙壁 (High Ground Wall) at X=20
  // User Requested: Wall from (20, 16.1) to (20, 127.9)
  // Thickness 1.0" -> X: 19.5 to 20.5 (Centered at 20)
  // Height: 127.9 - 16.1 = 111.8
  add_obstacle_inch(19.5f, 16.1f, 1.0f, 111.8f, COST_OBSTACLE,
                    ROBOT_RADIUS_INCH);

  // 南侧坡道: Y: 16.1 - 54.0
  add_obstacle_inch(0.0f, 16.1f, 19.0f, 37.9f, COST_RAMP);

  // 中央平台: Y: 54.0 - 90.0
  add_obstacle_inch(0.0f, 54.0f, 19.0f, 36.0f, COST_RAMP);

  // 北侧坡道: Y: 90.0 - 127.9
  add_obstacle_inch(0.0f, 90.0f, 19.0f, 37.9f, COST_RAMP);

  // ========== 3. 基地 (Nexus) 8"x5" ==========
  ESP_LOGI(TAG, "  Placing nexus bases (Inflated)...");
  // 南基地: 中心(30, 2.5) -> X:[26,34], Y:[0,5]
  add_obstacle_inch(26.0f, 0.0f, 8.0f, 5.0f, COST_OBSTACLE, ROBOT_RADIUS_INCH);
  // 北基地: 中心(30, 141.5) -> X:[26,34], Y:[139,144]
  add_obstacle_inch(26.0f, 139.0f, 8.0f, 5.0f, COST_OBSTACLE,
                    ROBOT_RADIUS_INCH);

  // ========== 4. 塔楼 (Tower) 6"x8" ==========
  ESP_LOGI(TAG, "  Placing towers (Inflated)...");

  // 低地塔: 中心(40, 72), 底座约6x8
  // 占用区域: X:[37, 43], Y:[68, 76]
  add_obstacle_inch(37.0f, 68.0f, 6.0f, 8.0f, COST_TOWER, ROBOT_RADIUS_INCH);
  // 为了安全，Tower区域设为 OBSTACLE (不可通行)
  add_obstacle_inch(37.0f, 68.0f, 6.0f, 8.0f, COST_OBSTACLE, ROBOT_RADIUS_INCH);

  // 高地塔: 中心(9.5, 72)
  // X:[6.5, 12.5], Y:[68, 76]
  add_obstacle_inch(6.5f, 68.0f, 6.0f, 8.0f, COST_OBSTACLE, ROBOT_RADIUS_INCH);

  // 南侧基地: 中心(40, 2.5), 占用 X:[36,44], Y:[0,5]
  add_obstacle_inch(36.0f, 0.0f, 8.0f, 5.0f, COST_OBSTACLE);

  // 北侧基地: 中心(40, 141.5), 占用 X:[36,44], Y:[139,144]
  add_obstacle_inch(36.0f, 139.0f, 8.0f, 5.0f, COST_OBSTACLE);

  // ========== 4. 塔楼 (Tower) 6"x8" ==========
  ESP_LOGI(TAG, "  Placing tower...");

  // 低地塔: 中心(40, 72), 底座约6x8
  // 占用区域: X:[37, 43], Y:[68, 76]
  add_obstacle_inch(37.0f, 68.0f, 6.0f, 8.0f, COST_TOWER);

  g_map_initialized = true;
  ESP_LOGI(TAG, "ROBA 2025 grid map initialized successfully");

  return ESP_OK;
}

/**
 * @brief 获取指定位置的cost值
 * @param x 物理X坐标 (0-60")
 * @param y 物理Y坐标 (0-144")
 */
uint8_t grid_map_get_cost(int16_t x, int16_t y) {
  // 物理坐标范围: X:[0,60), Y:[0,144)
  if (x < 0 || x >= 60 || y < 0 || y >= 144) {
    return COST_OBSTACLE; // 越界视为障碍
  }
  return g_grid_map[y][x];
}

/**
 * @brief 英寸坐标转换为像素坐标
 * 物理坐标: X(0-60"), Y(0-144")
 */
void grid_map_inch_to_pixel(float inch_x, float inch_y, int16_t *pixel_x,
                            int16_t *pixel_y) {
  *pixel_x = (int16_t)(inch_x * MAP_RESOLUTION);
  *pixel_y = (int16_t)(inch_y * MAP_RESOLUTION);
}

/**
 * @brief 像素坐标转换为英寸坐标
 */
void grid_map_pixel_to_inch(int16_t pixel_x, int16_t pixel_y, float *inch_x,
                            float *inch_y) {
  *inch_x = (float)pixel_x / MAP_RESOLUTION;
  *inch_y = (float)pixel_y / MAP_RESOLUTION;
}

/**
 * @brief Vive坐标转换为像素坐标
 *
 * Vive范围: 0-8191
 * 物理地图: X(0-60"), Y(0-144")
 */
void grid_map_vive_to_pixel(uint16_t vive_x, uint16_t vive_y, int16_t *pixel_x,
                            int16_t *pixel_y) {
  // Calibrated Affine Mapping:
  // P_inch = A * V + T
  float inch_x, inch_y;

  if (g_is_calibrated) {
    inch_x = g_calib_a * (float)vive_x + g_calib_b * (float)vive_y + g_calib_c;
    inch_y = g_calib_d * (float)vive_x + g_calib_e * (float)vive_y + g_calib_f;
  } else {
    // Fallback or uncalibrated (simple linear scaling)
    // Adjust these defaults if necessary or ensure calibration is run
    inch_x = (float)vive_x * 60.0f / 8192.0f;
    inch_y = (float)vive_y * 144.0f / 8192.0f;
  }

  // Map Inch -> Pixel
  grid_map_inch_to_pixel(inch_x, inch_y, pixel_x, pixel_y);

  // Boundary check handled in inch_to_pixel implicitly by casting, but explicit
  // clamping is safer
  if (*pixel_x < 0)
    *pixel_x = 0;
  if (*pixel_x >= 60)
    *pixel_x = 59;
  if (*pixel_y < 0)
    *pixel_y = 0;
  if (*pixel_y >= 144)
    *pixel_y = 143;
}

/**
 * @brief 像素坐标转换为Vive坐标
 */
void grid_map_pixel_to_vive(int16_t pixel_x, int16_t pixel_y, uint16_t *vive_x,
                            uint16_t *vive_y) {
  // 线性映射: Physical (0-60 or 0-144) -> Vive (0-8191)
  *vive_x = (uint16_t)((float)pixel_x * 8192.0f / 60.0f);
  *vive_y = (uint16_t)((float)pixel_y * 8192.0f / 144.0f);

  // 边界限制
  if (*vive_x > 8191)
    *vive_x = 8191;
  if (*vive_y > 8191)
    *vive_y = 8191;
}

/**
 * @brief 检查位置是否可通行
 */
bool grid_map_is_free(int16_t x, int16_t y) {
  return grid_map_get_cost(x, y) < COST_OBSTACLE;
}

/**
 * @brief 清理栅格地图，释放内存
 */
void grid_map_deinit(void) {
  if (g_grid_map != NULL) {
    free(g_grid_map);
    g_grid_map = NULL;
    g_map_initialized = false;
    ESP_LOGI(TAG, "Grid map memory freed");
  }
}
