/**
 * @file grid_map.cpp
 * @brief MEAM 5100 ROBA Field Grid Map Implementation
 */

#include "include/grid_map.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <Arduino.h>
#include <string.h>

static const char *TAG = "GRID_MAP";

// 全局地图数组指针 (288 x 120 = 34,560 bytes)
// 使用动态分配避免静态内存不足
static uint8_t (*g_grid_map)[MAP_WIDTH] = NULL;
static bool g_map_initialized = false;

/**
 * @brief 辅助函数：添加障碍物（英寸坐标）
 */
static void add_obstacle_inch(float x_in, float y_in, float w_in, float h_in, uint8_t cost)
{
    int16_t x_start = (int16_t)(x_in * MAP_RESOLUTION);
    int16_t y_start = (int16_t)(y_in * MAP_RESOLUTION);
    int16_t w_pixel = (int16_t)(w_in * MAP_RESOLUTION);
    int16_t h_pixel = (int16_t)(h_in * MAP_RESOLUTION);

    for (int16_t x = x_start; x < x_start + w_pixel; x++) {
        for (int16_t y = y_start; y < y_start + h_pixel; y++) {
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
                g_grid_map[y][x] = cost;
            }
        }
    }
}

/**
 * @brief 初始化栅格地图
 */
esp_err_t grid_map_init(void)
{
    if (g_map_initialized) {
        ESP_LOGW(TAG, "Grid map already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing grid map...");
    ESP_LOGI(TAG, "  Field size: %d\" x %d\"", FIELD_LENGTH_INCH, FIELD_WIDTH_INCH);
    ESP_LOGI(TAG, "  Map size: %d x %d pixels", MAP_WIDTH, MAP_HEIGHT);
    ESP_LOGI(TAG, "  Resolution: %d pixels/inch (%.2f inch/pixel)",
             MAP_RESOLUTION, 1.0f / MAP_RESOLUTION);

    // 检查可用内存
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "  Free heap before allocation: %u bytes", free_heap);
    ESP_LOGI(TAG, "  Largest free block: %u bytes", largest_block);

    // 动态分配地图内存 (8,640 bytes - 从34,560减少75%)
    size_t map_size = MAP_HEIGHT * MAP_WIDTH * sizeof(uint8_t);
    g_grid_map = (uint8_t (*)[MAP_WIDTH])heap_caps_malloc(map_size, MALLOC_CAP_8BIT);

    if (g_grid_map == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes for grid map", map_size);
        ESP_LOGE(TAG, "  Free heap: %u bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));
        ESP_LOGE(TAG, "  Largest block: %u bytes", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "  Allocated %u bytes for grid map at address %p", map_size, g_grid_map);

    // 初始化为平地
    ESP_LOGI(TAG, "  Initializing map to COST_FREE...");
    memset(g_grid_map, COST_FREE, map_size);
    ESP_LOGI(TAG, "  Map initialization complete");

    // ========== 1. 绘制四周墙壁 ==========
    ESP_LOGI(TAG, "  Drawing walls...");
    float wall_thickness_in = 1.0f;
    int16_t wall_px = (int16_t)(wall_thickness_in * MAP_RESOLUTION);

    for (int16_t x = 0; x < MAP_WIDTH; x++) {
        for (int16_t y = 0; y < MAP_HEIGHT; y++) {
            // 左墙 or 右墙
            if (x < wall_px || x >= MAP_WIDTH - wall_px) {
                g_grid_map[y][x] = COST_OBSTACLE;
            }
            // 下墙 or 上墙
            if (y < wall_px || y >= MAP_HEIGHT - wall_px) {
                g_grid_map[y][x] = COST_OBSTACLE;
            }
        }
    }

    // ========== 2. 基地 (Nexus) ==========
    float nexus_w = 8.0f, nexus_h = 5.0f;

    // 蓝色基地 (左下角)
    add_obstacle_inch(1.0f, FIELD_WIDTH_INCH - nexus_h - 1.0f, nexus_w, nexus_h, COST_OBSTACLE);

    // 红色基地 (右上角)
    add_obstacle_inch(FIELD_LENGTH_INCH - nexus_w - 1.0f, 1.0f, nexus_w, nexus_h, COST_OBSTACLE);

    // ========== 3. 塔 (Towers) ==========
    float tower_w = 8.0f, tower_h = 6.0f;

    // 塔 1 (平地)
    add_obstacle_inch(40.0f, 15.0f, tower_w, tower_h, COST_TOWER);

    // 塔 2 (坡道上)
    add_obstacle_inch(100.0f, 40.0f, tower_w, tower_h, COST_TOWER);

    // ========== 4. 坡道区域 ==========
    // 坡道范围: X: 20" - 124", Y: 20" - 40"
    float ramp_start_x = 20.0f;
    float ramp_end_x = 124.0f;
    float ramp_start_y = 20.0f;
    float ramp_end_y = 40.0f;

    int16_t ramp_x_start_px = (int16_t)(ramp_start_x * MAP_RESOLUTION);
    int16_t ramp_x_end_px = (int16_t)(ramp_end_x * MAP_RESOLUTION);
    int16_t ramp_y_start_px = (int16_t)(ramp_start_y * MAP_RESOLUTION);
    int16_t ramp_y_end_px = (int16_t)(ramp_end_y * MAP_RESOLUTION);

    for (int16_t x = ramp_x_start_px; x < ramp_x_end_px; x++) {
        for (int16_t y = ramp_y_start_px; y < ramp_y_end_px; y++) {
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
                // 只有当前是平地时才设置为坡道（避免覆盖塔）
                if (g_grid_map[y][x] == COST_FREE) {
                    g_grid_map[y][x] = COST_RAMP;
                }
            }
        }
    }

    // ========== 5. 坡道护栏 ==========
    float rail_thickness = 1.0f;
    int16_t rail_thick_px = (int16_t)(rail_thickness * MAP_RESOLUTION);

    // 下护栏 (Y = 20")
    for (int16_t x = ramp_x_start_px; x < ramp_x_end_px; x++) {
        for (int16_t y = ramp_y_start_px; y < ramp_y_start_px + rail_thick_px; y++) {
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
                g_grid_map[y][x] = COST_OBSTACLE;
            }
        }
    }

    // 上护栏 (Y = 40")
    for (int16_t x = ramp_x_start_px; x < ramp_x_end_px; x++) {
        for (int16_t y = ramp_y_end_px - rail_thick_px; y < ramp_y_end_px; y++) {
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
                g_grid_map[y][x] = COST_OBSTACLE;
            }
        }
    }

    g_map_initialized = true;
    ESP_LOGI(TAG, "Grid map initialized successfully");

    return ESP_OK;
}

/**
 * @brief 获取指定位置的cost值
 */
uint8_t grid_map_get_cost(int16_t x, int16_t y)
{
    if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) {
        return COST_OBSTACLE;  // 越界视为障碍
    }
    return g_grid_map[y][x];
}


/**
 * @brief 英寸坐标转换为像素坐标
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
 * 假设Vive坐标系与地图坐标系一致
 * Vive范围: 0-8191
 * 地图范围: 0-288 (X), 0-120 (Y)
 */
void grid_map_vive_to_pixel(uint16_t vive_x, uint16_t vive_y, int16_t *pixel_x, int16_t *pixel_y)
{
    // 线性映射: Vive (0-8191) -> Map (0-288 or 0-120)
    *pixel_x = (int16_t)((float)vive_x * MAP_WIDTH / 8192.0f);
    *pixel_y = (int16_t)((float)vive_y * MAP_HEIGHT / 8192.0f);

    // 边界限制
    if (*pixel_x < 0) *pixel_x = 0;
    if (*pixel_x >= MAP_WIDTH) *pixel_x = MAP_WIDTH - 1;
    if (*pixel_y < 0) *pixel_y = 0;
    if (*pixel_y >= MAP_HEIGHT) *pixel_y = MAP_HEIGHT - 1;
}

/**
 * @brief 像素坐标转换为Vive坐标
 */
void grid_map_pixel_to_vive(int16_t pixel_x, int16_t pixel_y, uint16_t *vive_x, uint16_t *vive_y)
{
    // 线性映射: Map (0-288 or 0-120) -> Vive (0-8191)
    *vive_x = (uint16_t)((float)pixel_x * 8192.0f / MAP_WIDTH);
    *vive_y = (uint16_t)((float)pixel_y * 8192.0f / MAP_HEIGHT);

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
