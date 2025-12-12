/**
 * @file grid_map.h
 * @brief MEAM 5100 ROBA Field Grid Map
 *
 * 栅格地图定义 - 基于ROBA比赛场地
 * Grid map definition based on ROBA competition field
 *
 * 场地尺寸: 144" x 60" (长 x 宽)
 * 分辨率: 2 pixels/inch (0.5" per grid)
 * 地图尺寸: 288 x 120 pixels
 */

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "esp_err.h"
#include "nav_config.h" // Needed for nav_calib_point_t definition
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 地图参数 / Map Parameters ========== */

/**
 * 场地物理尺寸 (英寸)
 * Field physical dimensions (inches)
 */
#define FIELD_LENGTH_INCH 144
#define FIELD_WIDTH_INCH 60

/**
 * 地图分辨率 (grids per inch)
 * Map resolution: 1 pixel/inch = 1.0 inch per grid
 * 降低分辨率以节省内存 (从2改为1，节省75%内存)
 */
#define MAP_RESOLUTION 1

/**
 * 地图像素尺寸
 * Map size in pixels
 * 新分辨率: 144 x 60 = 8,640 格子 (原来 288 x 120 = 34,560)
 */
#define MAP_WIDTH (FIELD_WIDTH_INCH * MAP_RESOLUTION)   // 60
#define MAP_HEIGHT (FIELD_LENGTH_INCH * MAP_RESOLUTION) // 144

/**
 * Cost值定义
 * Cost values for different terrain types
 */
#define COST_FREE 1       // 平地 / Free space
#define COST_RAMP 2       // 坡道 / Ramp
#define COST_TOWER 5      // 塔 / Tower
#define COST_OBSTACLE 255 // 障碍物 / Obstacle (墙、基地等)

/* ========== 数据结构 / Data Structures ========== */

/**
 * @brief 地图坐标点 (像素坐标)
 * Map coordinate point (pixel coordinates)
 */
typedef struct {
  int16_t x; // X坐标 (0-287)
  int16_t y; // Y坐标 (0-119)
} map_point_t;

/**
 * @brief 物理坐标点 (英寸)
 * Physical coordinate point (inches)
 */
typedef struct {
  float x; // X坐标 (英寸)
  float y; // Y坐标 (英寸)
} physical_point_t;

/* ========== API函数 / API Functions ========== */

/**
 * @brief 初始化栅格地图
 * Initialize grid map
 *
 * @return ESP_OK on success
 */
esp_err_t grid_map_init(void);

/**
 * @brief 获取指定位置的cost值
 * Get cost value at specified position
 *
 * @param x X坐标 (像素)
 * @param y Y坐标 (像素)
 * @return Cost值 (1-255)
 */
uint8_t grid_map_get_cost(int16_t x, int16_t y);

/**
 * @brief 检查位置是否可通行
 * Check if position is traversable
 *
 * @param x X坐标 (像素)
 * @param y Y坐标 (像素)
 * @return true if traversable, false if obstacle
 */
bool grid_map_is_free(int16_t x, int16_t y);

/**
 * @brief 英寸坐标转换为像素坐标
 * Convert inch coordinates to pixel coordinates
 *
 * @param inch_x X坐标 (英寸)
 * @param inch_y Y坐标 (英寸)
 * @param pixel_x 输出X坐标 (像素)
 * @param pixel_y 输出Y坐标 (像素)
 */
void grid_map_inch_to_pixel(float inch_x, float inch_y, int16_t *pixel_x,
                            int16_t *pixel_y);

/**
 * @brief 像素坐标转换为英寸坐标
 * Convert pixel coordinates to inch coordinates
 *
 * @param pixel_x X坐标 (像素)
 * @param pixel_y Y坐标 (像素)
 * @param inch_x 输出X坐标 (英寸)
 * @param inch_y 输出Y坐标 (英寸)
 */
void grid_map_pixel_to_inch(int16_t pixel_x, int16_t pixel_y, float *inch_x,
                            float *inch_y);

/**
 * @brief Vive坐标转换为像素坐标
 * Convert Vive coordinates to pixel coordinates
 *
 * @param vive_x Vive X坐标 (0-8191)
 * @param vive_y Vive Y坐标 (0-8191)
 * @param pixel_x 输出X坐标 (像素)
 * @param pixel_y 输出Y坐标 (像素)
 */
void grid_map_vive_to_pixel(uint16_t vive_x, uint16_t vive_y, int16_t *pixel_x,
                            int16_t *pixel_y);

/**
 * @brief 像素坐标转换为Vive坐标
 * Convert pixel coordinates to Vive coordinates
 *
 * @param pixel_x X坐标 (像素)
 * @param pixel_y Y坐标 (像素)
 * @param vive_x 输出Vive X坐标
 * @param vive_y 输出Vive Y坐标
 */
void grid_map_pixel_to_vive(int16_t pixel_x, int16_t pixel_y, uint16_t *vive_x,
                            uint16_t *vive_y);

/**
 * @brief 清理栅格地图，释放内存
 * Deinitialize grid map and free memory
 */
void grid_map_deinit(void);

/**
 * @brief 使用标定点计算并应用 Affine Transform 参数
 */
esp_err_t grid_map_calibrate_affine(const nav_calib_point_t *points, int count);

#ifdef __cplusplus
}
#endif

#endif // GRID_MAP_H
