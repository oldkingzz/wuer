#!/usr/bin/env python3
"""
检查路径是否与障碍物碰撞
"""

import numpy as np

# 车辆参数
ROBOT_RADIUS = 5.0  # inches

# 障碍物定义
obstacles = [
    # 南侧基地: X:[26,34], Y:[0,5]
    {'name': 'South Nexus', 'x_min': 26, 'x_max': 34, 'y_min': 0, 'y_max': 5},
    # 北侧基地: X:[26,34], Y:[139,144]
    {'name': 'North Nexus', 'x_min': 26, 'x_max': 34, 'y_min': 139, 'y_max': 144},
    # 高地塔: X:[6.5, 12.5], Y:[68, 76]
    {'name': 'High Tower', 'x_min': 6.5, 'x_max': 12.5, 'y_min': 68, 'y_max': 76},
    # 低地塔: X:[37, 43], Y:[68, 76]
    {'name': 'Low Tower', 'x_min': 37, 'x_max': 43, 'y_min': 68, 'y_max': 76},
]

def check_point_collision(x, y, radius):
    """检查点(x,y)加上半径radius是否与障碍物碰撞"""
    collisions = []
    for obs in obstacles:
        # 计算点到矩形的最短距离
        dx = max(obs['x_min'] - x, 0, x - obs['x_max'])
        dy = max(obs['y_min'] - y, 0, y - obs['y_max'])
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance < radius:
            collisions.append({
                'obstacle': obs['name'],
                'position': (x, y),
                'distance': distance,
                'clearance': radius - distance
            })
    return collisions

# 测试当前路径中靠近基地的点
print("=" * 80)
print("检查路径点是否与障碍物碰撞")
print("=" * 80)

# 南侧基地绕行路径（最新版本）
south_path = [
    (39, 8),      # 接近基地右侧（远离基地）
    (39, 10),     # 向上到安全Y坐标
    (34, 10),     # 基地右边缘
    (30, 10),     # 基地中央
    (26, 10),     # 基地左边缘
    (21, 10),     # 离开基地（远离基地）
    (21, 8),      # 向下回到下边缘
]

print("\n南侧基地绕行路径:")
print(f"基地范围: X:[26,34], Y:[0,5]")
print(f"车辆半径: {ROBOT_RADIUS}\"")
print("-" * 80)

for x, y in south_path:
    collisions = check_point_collision(x, y, ROBOT_RADIUS)
    if collisions:
        print(f"❌ 点 ({x}, {y}) 碰撞!")
        for col in collisions:
            print(f"   - {col['obstacle']}: 距离={col['distance']:.2f}\", 侵入={col['clearance']:.2f}\"")
    else:
        # 计算到基地的最近距离
        obs = obstacles[0]  # 南侧基地
        dx = max(obs['x_min'] - x, 0, x - obs['x_max'])
        dy = max(obs['y_min'] - y, 0, y - obs['y_max'])
        distance = np.sqrt(dx**2 + dy**2)
        clearance = distance - ROBOT_RADIUS
        print(f"✅ 点 ({x}, {y}) 安全 - 到基地距离={distance:.2f}\", 余量={clearance:.2f}\"")

# 北侧基地绕行路径（最新版本）
north_path = [
    (21, 136),      # 接近基地左侧（远离基地）
    (21, 134),      # 向下到安全Y坐标
    (26, 134),      # 基地左边缘
    (30, 134),      # 基地中央
    (34, 134),      # 基地右边缘
    (39, 134),      # 离开基地（远离基地）
    (39, 136),      # 向上回到上边缘
]

print("\n北侧基地绕行路径:")
print(f"基地范围: X:[26,34], Y:[139,144]")
print(f"车辆半径: {ROBOT_RADIUS}\"")
print("-" * 80)

for x, y in north_path:
    collisions = check_point_collision(x, y, ROBOT_RADIUS)
    if collisions:
        print(f"❌ 点 ({x}, {y}) 碰撞!")
        for col in collisions:
            print(f"   - {col['obstacle']}: 距离={col['distance']:.2f}\", 侵入={col['clearance']:.2f}\"")
    else:
        # 计算到基地的最近距离
        obs = obstacles[1]  # 北侧基地
        dx = max(obs['x_min'] - x, 0, x - obs['x_max'])
        dy = max(obs['y_min'] - y, 0, y - obs['y_max'])
        distance = np.sqrt(dx**2 + dy**2)
        clearance = distance - ROBOT_RADIUS
        print(f"✅ 点 ({x}, {y}) 安全 - 到基地距离={distance:.2f}\", 余量={clearance:.2f}\"")

print("\n" + "=" * 80)
print("建议：")
print("- 南侧基地：车中心Y坐标应该 >= 10 (基地顶部5 + 车半径5)")
print("- 北侧基地：车中心Y坐标应该 <= 134 (基地底部139 - 车半径5)")
print("=" * 80)

