/**
 * @file wall_following_v2.h
 * @brief Wall Following V2 - No FreeRTOS Task, Blocking Execution
 *
 * 使用同步 ToF 读取 + 编码器里程计，完成绕墙任务
 */

#ifndef WALL_FOLLOWING_V2_H
#define WALL_FOLLOWING_V2_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ========== 状态定义 ==========

typedef enum {
  WF_STATE_IDLE,          // 未运行
  WF_STATE_FOLLOW_WALL,   // 沿墙行驶
  WF_STATE_CONVEX_CORNER, // 凸角处理 (右墙消失)
  WF_STATE_FRONT_BLOCKED, // 前方受阻 (需左转)
  WF_STATE_RAMP,          // 坡道处理
  WF_STATE_DONE,          // 完成
  WF_STATE_ERROR          // 错误
} wf_state_t;

typedef struct {
  wf_state_t state;
  uint16_t tof_front_mm; // 前方 ToF 距离
  uint16_t tof_right_mm; // 右侧 ToF 距离
  float odo_x_m;         // 里程计 X (m)
  float odo_y_m;         // 里程计 Y (m)
  float odo_heading_rad; // 里程计航向 (rad)
  uint32_t elapsed_ms;   // 运行时间
  bool is_running;       // 是否正在运行
} wf_status_t;

// ========== 配置参数 (V2 Oscillation) ==========

// === 传感器偏移补偿 ===
// 前方 ToF 到轮子边缘: 5cm (50mm)
#define WF_FRONT_TOF_OFFSET_MM 50
// 右侧 ToF 到轮子边缘: 1cm (10mm)
#define WF_RIGHT_TOF_OFFSET_MM 10

// === 距离阈值 ===

// 期望离墙距离 (mm) - ToF 读数目标
// 保持较远距离以防震荡撞墙
// 期望离墙距离 (mm)
#define WF_TARGET_WALL_DIST_MM 200

// 右侧紧急避障阈值
#define WF_RIGHT_EMERGENCY_MM 40

// 凸角检测阈值
#define WF_CONVEX_THRESHOLD_MM 450

// 前方障碍物阈值：检测前方墙壁/障碍 -> 触发左转
#define WF_FRONT_STOP_MM 200

// 前方紧急回退阈值 -> 触发倒车
#define WF_FRONT_BACKUP_MM 130

// === 运动参数 ===

// 基础线速度 (m/s)
#define WF_BASE_SPEED 0.15f

// 回退速度 (m/s)
#define WF_BACKUP_SPEED 0.15f

// 最大角速度 (rad/s)
#define WF_MAX_ANGULAR 2.0f

// === 摆动模式参数 ===

// 摆动幅度 (rad/s) - 增大以覆盖约 23 度 (0.4 rad) 的盲区
// Calculation: Angle = Amp / (2 * PI * Freq) => 1.5 / (3.77) = 0.4 rad
#define WF_OSCILLATION_AMPLITUDE 1.5f

// 摆动频率 (Hz) - 降低频率以允许更大的摆动角度
// 太快会导致电机来不及转到最大角度
#define WF_OSCILLATION_FREQ 0.6f

// 循环周期 (ms) - 需要足够快来执行摆动
#define WF_LOOP_PERIOD_MS 20

// ========== API ==========

/**
 * @brief 初始化 Wall Following 模块
 * @return ESP_OK on success
 */
esp_err_t wall_following_v2_init(void);

/**
 * @brief 启动 Wall Following (阻塞执行)
 *
 * 此函数会阻塞直到任务完成或被停止。
 * 调用方应在单独的任务或主循环中调用。
 *
 * @return ESP_OK on success
 */
esp_err_t wall_following_v2_run_blocking(void);

/**
 * @brief 请求停止 Wall Following
 *
 * 非阻塞调用，设置停止标志。
 * run_blocking 会在下一个循环检测到并退出。
 */
void wall_following_v2_request_stop(void);

/**
 * @brief 获取当前状态
 */
esp_err_t wall_following_v2_get_status(wf_status_t *status);

// ========== 兼容旧接口 (保留给 web 控制) ==========

esp_err_t wall_following_v2_start(void);
esp_err_t wall_following_v2_stop(void);

#ifdef __cplusplus
}
#endif

#endif // WALL_FOLLOWING_V2_H
