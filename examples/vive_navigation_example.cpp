/**
 * @file vive_navigation_example.cpp
 * @brief Vive导航系统使用示例 / Vive Navigation System Usage Example
 * 
 * 这个文件展示如何使用Vive导航系统
 * This file demonstrates how to use the Vive navigation system
 * 
 * 注意：这不是一个完整的Arduino程序，只是示例代码
 * Note: This is not a complete Arduino program, just example code
 */

#include "../src/include/vive_navigation.h"
#include "../src/include/vive_sensor.h"
#include "../src/include/chassis.h"

/**
 * 示例1: 基本导航
 * Example 1: Basic Navigation
 */
void example_basic_navigation()
{
    // 初始化导航系统
    vive_nav_init();
    
    // 设置目标点
    vive_nav_set_target(4000, 3000);
    
    // 开始导航
    vive_nav_start();
    
    // 等待到达
    while (1) {
        nav_status_t status;
        vive_nav_get_status(&status);
        
        if (status.state == NAV_STATE_ARRIVED) {
            Serial.println("到达目标点！");
            break;
        } else if (status.state == NAV_STATE_ERROR) {
            Serial.println("导航错误！");
            break;
        }
        
        delay(100);
    }
    
    // 停止导航
    vive_nav_stop();
}

/**
 * 示例2: 监控导航状态
 * Example 2: Monitor Navigation Status
 */
void example_monitor_status()
{
    nav_status_t status;
    vive_nav_get_status(&status);
    
    Serial.println("========== Navigation Status ==========");
    
    // 打印状态
    Serial.print("State: ");
    switch (status.state) {
        case NAV_STATE_IDLE:       Serial.println("IDLE"); break;
        case NAV_STATE_NAVIGATING: Serial.println("NAVIGATING"); break;
        case NAV_STATE_ARRIVED:    Serial.println("ARRIVED"); break;
        case NAV_STATE_ERROR:      Serial.println("ERROR"); break;
    }
    
    // 打印位姿
    Serial.printf("Current Pose: (%d, %d) Heading: %.1f°\n",
                 status.current_pose.x, 
                 status.current_pose.y,
                 status.current_pose.heading);
    
    // 打印目标
    Serial.printf("Target: (%d, %d)\n",
                 status.target.x, 
                 status.target.y);
    
    // 打印距离和误差
    Serial.printf("Distance: %.1f\n", status.distance_to_target);
    Serial.printf("Heading Error: %.1f°\n", status.heading_error);
    
    // 打印速度
    Serial.printf("Velocity: Linear=%.2f m/s, Angular=%.2f rad/s\n",
                 status.linear_velocity,
                 status.angular_velocity);
    
    Serial.println("======================================");
}

/**
 * 示例3: 多点导航
 * Example 3: Multi-Point Navigation
 */
void example_multi_point_navigation()
{
    // 定义路径点
    vive_point_t waypoints[] = {
        {3000, 2000},
        {4000, 2000},
        {4000, 3000},
        {3000, 3000}
    };
    
    int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);
    
    // 依次导航到每个点
    for (int i = 0; i < num_waypoints; i++) {
        Serial.printf("导航到路径点 %d: (%d, %d)\n", 
                     i + 1, 
                     waypoints[i].x, 
                     waypoints[i].y);
        
        // 设置目标并开始
        vive_nav_set_target(waypoints[i].x, waypoints[i].y);
        vive_nav_start();
        
        // 等待到达
        while (1) {
            nav_status_t status;
            vive_nav_get_status(&status);
            
            if (status.state == NAV_STATE_ARRIVED) {
                Serial.printf("到达路径点 %d\n", i + 1);
                break;
            } else if (status.state == NAV_STATE_ERROR) {
                Serial.printf("导航到路径点 %d 失败\n", i + 1);
                return;
            }
            
            delay(100);
        }
        
        // 在路径点停留1秒
        delay(1000);
    }
    
    Serial.println("完成所有路径点导航！");
}

/**
 * 示例4: 获取当前位姿
 * Example 4: Get Current Pose
 */
void example_get_pose()
{
    vive_pose_t pose;
    
    if (vive_nav_get_pose(&pose) == ESP_OK) {
        if (pose.valid) {
            Serial.printf("当前位置: (%d, %d)\n", pose.x, pose.y);
            Serial.printf("当前朝向: %.1f°\n", pose.heading);
        } else {
            Serial.println("位姿无效 - 检查Vive传感器");
        }
    }
}

/**
 * 示例5: 条件导航
 * Example 5: Conditional Navigation
 */
void example_conditional_navigation()
{
    // 设置目标
    vive_nav_set_target(4000, 3000);
    vive_nav_start();
    
    // 导航过程中监控状态
    while (1) {
        nav_status_t status;
        vive_nav_get_status(&status);
        
        // 如果距离太远，停止导航
        if (status.distance_to_target > 2000) {
            Serial.println("目标太远，停止导航");
            vive_nav_stop();
            break;
        }
        
        // 如果到达，退出
        if (status.state == NAV_STATE_ARRIVED) {
            Serial.println("到达目标");
            break;
        }
        
        // 如果错误，退出
        if (status.state == NAV_STATE_ERROR) {
            Serial.println("导航错误");
            break;
        }
        
        delay(100);
    }
}

