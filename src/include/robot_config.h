/**
 * @file robot_config.h
 * @brief 全局机器人配置参数（例如统一的线速度）
 *
 * 这里定义一些在整车各个模块中都需要用到的基础常量，
 * 比如“默认行驶线速度”。你以后只需要改这里的数值，
 * 摇杆手动控制、寻墙轨迹跟随、导航模块都会一起跟着变化。
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 机器人默认行驶线速度 (m/s)
 *
 * 当前值：0.30 m/s
 *
 * 用途：
 * - 网页摇杆的最大线速度（MAX_LINEAR_VEL）
 * - 寻墙/预定义轨迹巡航的前进速度（FORWARD_SPEED）
 * - 导航模块的最大线速度（NAV_MAX_LINEAR_VELOCITY）
 *
 * 以后如果你想整体调快/调慢，只需要修改这个值。
 *
 * 注意：这里不用 `0.30f`，而是用 `0.30`，这样在 web.h 里做字符串拼接
 *       生成 JavaScript 常量时不会带上 `f` 后缀，避免 JS 语法错误。
 */
#define ROBOT_BASE_LINEAR_SPEED  0.1

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONFIG_H
