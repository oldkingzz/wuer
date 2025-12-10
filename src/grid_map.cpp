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
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <Arduino.h>
#include <string.h>

static const char *TAG = "GRID_MAP";

// 全局地图数组指针
// 注意: 实际地图尺寸是 60(宽) x 144(长)
// 但是 MAP_WIDTH=144, MAP_HEIGHT=60 (为了兼容头文件定义)
// 所以我们需要转置坐标: 物理(x,y) -> 数组[x][y]
static uint8_t (*g_grid_map)[MAP_WIDTH] = NULL;
static bool g_map_initialized = false;

/**
 * @brief 辅助函数：添加障碍物（英寸坐标）
 * @param x_in 物理X坐标 (0-60")
 * @param y_in 物理Y坐标 (0-144")
 * @param w_in 宽度 (X方向)
 * @param h_in 高度 (Y方向)
 */
static void add_obstacle_inch(float x_in, float y_in, float w_in, float h_in, uint8_t cost)
{
    int16_t x_start = (int16_t)(x_in * MAP_RESOLUTION);
    int16_t y_start = (int16_t)(y_in * MAP_RESOLUTION);
    int16_t w_pixel = (int16_t)(w_in * MAP_RESOLUTION);
    int16_t h_pixel = (int16_t)(h_in * MAP_RESOLUTION);

    // 物理坐标: X(0-60), Y(0-144)
    // 数组索引: g_grid_map[x][y] 因为我们转置了
    for (int16_t x = x_start; x < x_start + w_pixel && x < 60; x++) {
        for (int16_t y = y_start; y < y_start + h_pixel && y < 144; y++) {
            if (x >= 0 && y >= 0) {
                g_grid_map[x][y] = cost;
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
esp_err_t grid_map_init(void)
{
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
    ESP_LOGI(TAG, "  Free heap: %u bytes, Largest block: %u bytes", free_heap, largest_block);

    // 动态分配地图内存: 60 x 144 = 8,640 bytes
    size_t map_size = 60 * 144 * sizeof(uint8_t);
    g_grid_map = (uint8_t (*)[MAP_WIDTH])heap_caps_malloc(map_size, MALLOC_CAP_8BIT);

    if (g_grid_map == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes for grid map", map_size);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "  Allocated %u bytes at %p", map_size, g_grid_map);

    // 初始化为平地
    memset(g_grid_map, COST_FREE, map_size);

    // ========== 1. 场地边界墙壁 ==========
    ESP_LOGI(TAG, "  Drawing boundary walls...");
    // X=0, X=60, Y=0, Y=144
    for (int16_t y = 0; y < 144; y++) {
        g_grid_map[0][y] = COST_OBSTACLE;      // X=0 左墙
        g_grid_map[59][y] = COST_OBSTACLE;     // X=60 右墙
    }
    for (int16_t x = 0; x < 60; x++) {
        g_grid_map[x][0] = COST_OBSTACLE;      // Y=0 南墙
        g_grid_map[x][143] = COST_OBSTACLE;    // Y=144 北墙
    }

    // ========== 2. 高地结构 (X: 0-19) ==========
    ESP_LOGI(TAG, "  Drawing high level structure...");

    // 南侧坡道: Y: 16.1 - 54.0
    add_obstacle_inch(0.0f, 16.1f, 19.0f, 37.9f, COST_RAMP);

    // 中央平台: Y: 54.0 - 90.0
    add_obstacle_inch(0.0f, 54.0f, 19.0f, 36.0f, COST_RAMP);

    // 北侧坡道: Y: 90.0 - 127.9
    add_obstacle_inch(0.0f, 90.0f, 19.0f, 37.9f, COST_RAMP);

    // ========== 3. 基地 (Nexus) 8"x5" ==========
    ESP_LOGI(TAG, "  Placing nexus bases...");

    // 南侧基地: 中心(30, 2.5), 占用 X:[26,34], Y:[0,5]
    add_obstacle_inch(26.0f, 0.0f, 8.0f, 5.0f, COST_OBSTACLE);

    // 北侧基地: 中心(30, 141.5), 占用 X:[26,34], Y:[139,144]
    add_obstacle_inch(26.0f, 139.0f, 8.0f, 5.0f, COST_OBSTACLE);

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
uint8_t grid_map_get_cost(int16_t x, int16_t y)
{
    // 物理坐标范围: X:[0,60), Y:[0,144)
    if (x < 0 || x >= 60 || y < 0 || y >= 144) {
        return COST_OBSTACLE;  // 越界视为障碍
    }
    return g_grid_map[x][y];
}


/**
 * @brief 英寸坐标转换为像素坐标
 * 物理坐标: X(0-60"), Y(0-144")
 */
void grid_map_inch_to_pixel(float inch_x, float inch_y, int16_t *pixel_x, int16_t *pixel_y)
{
    *pixel_x = (int16_t)(inch_x * MAP_RESOLUTION);
    *pixel_y = (int16_t)(inch_y * MAP_RESOLUTION);
}

/**
 * @brief 像素坐标转换为英寸坐标
 */
void grid_map_pixel_to_inch(int16_t pixel_x, int16_t pixel_y, float *inch_x, float *inch_y)
{
    *inch_x = (float)pixel_x / MAP_RESOLUTION;
    *inch_y = (float)pixel_y / MAP_RESOLUTION;
}

/**
 * @brief Vive坐标转换为像素坐标
 *
 * Vive范围: 0-8191
 * 物理地图: X(0-60"), Y(0-144")
 */
void grid_map_vive_to_pixel(uint16_t vive_x, uint16_t vive_y, int16_t *pixel_x, int16_t *pixel_y)
{
    // 线性映射: Vive (0-8191) -> Physical (0-60 or 0-144)
    *pixel_x = (int16_t)((float)vive_x * 60.0f / 8192.0f);
    *pixel_y = (int16_t)((float)vive_y * 144.0f / 8192.0f);

    // 边界限制
    if (*pixel_x < 0) *pixel_x = 0;
    if (*pixel_x >= 60) *pixel_x = 59;
    if (*pixel_y < 0) *pixel_y = 0;
    if (*pixel_y >= 144) *pixel_y = 143;
}

/**
 * @brief 像素坐标转换为Vive坐标
 */
void grid_map_pixel_to_vive(int16_t pixel_x, int16_t pixel_y, uint16_t *vive_x, uint16_t *vive_y)
{
    // 线性映射: Physical (0-60 or 0-144) -> Vive (0-8191)
    *vive_x = (uint16_t)((float)pixel_x * 8192.0f / 60.0f);
    *vive_y = (uint16_t)((float)pixel_y * 8192.0f / 144.0f);

    // 边界限制
    if (*vive_x > 8191) *vive_x = 8191;
    if (*vive_y > 8191) *vive_y = 8191;
}

/**
 * @brief 检查位置是否可通行
 */
bool grid_map_is_free(int16_t x, int16_t y)
{
    return grid_map_get_cost(x, y) < COST_OBSTACLE;
}

/**
 * @brief 清理栅格地图，释放内存
 */
void grid_map_deinit(void)
{
    if (g_grid_map != NULL) {
        free(g_grid_map);
        g_grid_map = NULL;
        g_map_initialized = false;
        ESP_LOGI(TAG, "Grid map memory freed");
    }
}
