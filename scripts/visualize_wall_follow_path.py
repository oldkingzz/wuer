#!/usr/bin/env python3
"""
可视化寻墙路径
Visualize wall-following path on ROBA 2025 field
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# ========== 地图参数 ==========
FIELD_WIDTH = 60    # inches (X方向)
FIELD_LENGTH = 144  # inches (Y方向)

# ========== 车辆参数 ==========
# 实际整车直径约 11"，几何碰撞半径取 5.5"
ROBOT_RADIUS = 5.5  # inches (碰撞半径)
SAFE_MARGIN = 3.0   # inches (安全余量)
WALL_CLEARANCE = ROBOT_RADIUS + SAFE_MARGIN  # 车中心距墙壁距离 ≈ 8.5"

# ToF传感器位置（相对车中心）
TOF_SIDE_OFFSET = 5.0   # 侧面ToF距车中心距离（左侧）
TOF_FRONT_OFFSET = 5.0  # 前方ToF距车中心距离（前方）

# ========== 障碍物定义 ==========
obstacles = []

# 边界墙壁（不画，只是参考）
# X=0, X=60, Y=0, Y=144

# 南侧基地: 中心(30, 2.5), 占用 X:[26,34], Y:[0,5]
obstacles.append(patches.Rectangle((26, 0), 8, 5, linewidth=2, edgecolor='red', facecolor='lightcoral', label='Nexus'))

# 北侧基地: 中心(30, 141.5), 占用 X:[26,34], Y:[139,144]
obstacles.append(patches.Rectangle((26, 139), 8, 5, linewidth=2, edgecolor='red', facecolor='lightcoral'))

# 高地塔: 中心(9.5, 72), 底座 X:[6.5, 12.5], Y:[68, 76]
obstacles.append(patches.Rectangle((6.5, 68), 6, 8, linewidth=2, edgecolor='blue', facecolor='lightblue', label='Tower'))

# 低地塔: 中心(40, 72), 底座 X:[37, 43], Y:[68, 76]
obstacles.append(patches.Rectangle((37, 68), 6, 8, linewidth=2, edgecolor='blue', facecolor='lightblue'))

# 高地区域（坡道，浅色显示）
obstacles.append(patches.Rectangle((0, 16.1), 19, 37.9, linewidth=1, edgecolor='gray', facecolor='lightyellow', alpha=0.3, label='Ramp'))
obstacles.append(patches.Rectangle((0, 54.0), 19, 36.0, linewidth=1, edgecolor='gray', facecolor='lightyellow', alpha=0.3))
obstacles.append(patches.Rectangle((0, 90.0), 19, 37.9, linewidth=1, edgecolor='gray', facecolor='lightyellow', alpha=0.3))

# ========== 寻墙路径设计 ==========
# 策略：沿着地图边缘逆时针走一圈，必须经过所有凸角
# 车中心距墙壁 WALL_CLEARANCE = 8"

# 关键凸角位置（必须经过）
# 左下角: (0, 0)
# 左上角: (0, 144)
# 右上角: (60, 144)
# 右下角: (60, 0)

# 基地安全距离计算
# 南侧基地: Y:[0,5]，车半径≈5.5"，所以车中心Y >= 5 + ROBOT_RADIUS ≈ 10.5"
# 北侧基地: Y:[139,144]，车半径≈5.5"，所以车中心Y <= 139 - ROBOT_RADIUS ≈ 133.5"
SOUTH_NEXUS_SAFE_Y = 5 + ROBOT_RADIUS   # 南侧基地上方安全Y坐标
NORTH_NEXUS_SAFE_Y = 139 - ROBOT_RADIUS # 北侧基地下方安全Y坐标

waypoints = [
    # ========== 第1段：起点（左下角附近） ==========
    (WALL_CLEARANCE, 15),  # 起点

    # ========== 第2段：左边缘向上 ==========
    (WALL_CLEARANCE, 25),
    (WALL_CLEARANCE, 40),
    (WALL_CLEARANCE, 55),
    (WALL_CLEARANCE, 68),  # 接近高地塔
    (WALL_CLEARANCE, 76),  # 经过高地塔
    (WALL_CLEARANCE, 90),
    (WALL_CLEARANCE, 110),
    (WALL_CLEARANCE, 125),

    # ========== 第3段：左上角转弯 ==========
    (WALL_CLEARANCE, FIELD_LENGTH - WALL_CLEARANCE),  # 左上角
    (WALL_CLEARANCE + 3, FIELD_LENGTH - WALL_CLEARANCE),  # 转弯过渡

    # ========== 第4段：上边缘向右（绕开北侧基地） ==========
    (15, FIELD_LENGTH - WALL_CLEARANCE),
    (20, FIELD_LENGTH - WALL_CLEARANCE),

    # 绕过北侧基地（向下绕行，确保安全）
    # 基地范围: X:[26,34], Y:[139,144]
    # 车中心必须 Y <= NORTH_NEXUS_SAFE_Y = 139 - ROBOT_RADIUS
    (21, FIELD_LENGTH - WALL_CLEARANCE),      # 接近基地左侧（远离基地）
    (21, NORTH_NEXUS_SAFE_Y),                 # 向下到安全Y坐标
    (26, NORTH_NEXUS_SAFE_Y),                 # 基地左边缘
    (30, NORTH_NEXUS_SAFE_Y),                 # 基地中央
    (34, NORTH_NEXUS_SAFE_Y),                 # 基地右边缘
    (39, NORTH_NEXUS_SAFE_Y),                 # 离开基地（远离基地）
    (39, FIELD_LENGTH - WALL_CLEARANCE),      # 向上回到上边缘

    (42, FIELD_LENGTH - WALL_CLEARANCE),
    (48, FIELD_LENGTH - WALL_CLEARANCE),

    # ========== 第5段：右上角转弯 ==========
    (FIELD_WIDTH - WALL_CLEARANCE - 3, FIELD_LENGTH - WALL_CLEARANCE),  # 转弯过渡
    (FIELD_WIDTH - WALL_CLEARANCE, FIELD_LENGTH - WALL_CLEARANCE),  # 右上角
    (FIELD_WIDTH - WALL_CLEARANCE, FIELD_LENGTH - WALL_CLEARANCE - 3),  # 转弯过渡

    # ========== 第6段：右边缘向下 ==========
    (FIELD_WIDTH - WALL_CLEARANCE, 125),
    (FIELD_WIDTH - WALL_CLEARANCE, 110),
    (FIELD_WIDTH - WALL_CLEARANCE, 90),
    (FIELD_WIDTH - WALL_CLEARANCE, 76),  # 经过低地塔
    (FIELD_WIDTH - WALL_CLEARANCE, 68),  # 接近低地塔
    (FIELD_WIDTH - WALL_CLEARANCE, 55),
    (FIELD_WIDTH - WALL_CLEARANCE, 40),
    (FIELD_WIDTH - WALL_CLEARANCE, 25),
    (FIELD_WIDTH - WALL_CLEARANCE, 15),

    # ========== 第7段：右下角转弯 ==========
    (FIELD_WIDTH - WALL_CLEARANCE, WALL_CLEARANCE + 3),  # 转弯过渡
    (FIELD_WIDTH - WALL_CLEARANCE, WALL_CLEARANCE),  # 右下角
    (FIELD_WIDTH - WALL_CLEARANCE - 3, WALL_CLEARANCE),  # 转弯过渡

    # ========== 第8段：下边缘向左（绕开南侧基地） ==========
    (48, WALL_CLEARANCE),
    (42, WALL_CLEARANCE),

    # 绕过南侧基地（向上绕行，确保安全）
    # 基地范围: X:[26,34], Y:[0,5]
    # 车中心必须 Y >= SOUTH_NEXUS_SAFE_Y = 5 + ROBOT_RADIUS
    (39, WALL_CLEARANCE),                     # 接近基地右侧（远离基地）
    (39, SOUTH_NEXUS_SAFE_Y),                 # 向上到安全Y坐标
    (34, SOUTH_NEXUS_SAFE_Y),                 # 基地右边缘
    (30, SOUTH_NEXUS_SAFE_Y),                 # 基地中央
    (26, SOUTH_NEXUS_SAFE_Y),                 # 基地左边缘
    (21, SOUTH_NEXUS_SAFE_Y),                 # 离开基地（远离基地）
    (21, WALL_CLEARANCE),                     # 向下回到下边缘

    (20, WALL_CLEARANCE),
    (15, WALL_CLEARANCE),

    # ========== 第9段：左下角转弯 ==========
    (WALL_CLEARANCE + 3, WALL_CLEARANCE),  # 转弯过渡
    (WALL_CLEARANCE, WALL_CLEARANCE),  # 左下角
    (WALL_CLEARANCE, WALL_CLEARANCE + 3),  # 转弯过渡

    # 回到起点
    (WALL_CLEARANCE, 15),
]

# ========== 绘图 ==========
fig, ax = plt.subplots(figsize=(10, 20))

# 绘制地图边界
ax.add_patch(patches.Rectangle((0, 0), FIELD_WIDTH, FIELD_LENGTH,
                               linewidth=3, edgecolor='black', facecolor='white'))

# 绘制障碍物
for obs in obstacles:
    ax.add_patch(obs)

# 绘制路径
path_x = [wp[0] for wp in waypoints]
path_y = [wp[1] for wp in waypoints]
ax.plot(path_x, path_y, 'g-', linewidth=2, label='Path (Robot Center)', zorder=5)

# 绘制路径点
ax.plot(path_x, path_y, 'go', markersize=4, zorder=6)

# 标注设计起点（左下区域）和终点
ax.plot(waypoints[0][0], waypoints[0][1], 'r*', markersize=18, label='Design Start', zorder=7)
ax.plot(waypoints[-1][0], waypoints[-1][1], 'b*', markersize=18, label='End', zorder=7)

# ========= 标注运行时实际起点（根据 ToF 初始定位，右上角内圈） =========
# 在 C 代码中，calc_init_pos() 假设机器人在右上角，前/右 ToF 贴墙，
# 然后 find_closest_waypoint() 会选离 (FIELD_WIDTH - WALL_CLEARANCE,
# FIELD_LENGTH - WALL_CLEARANCE) 最近的路径点作为 g_start_wp_idx。
runtime_start = (FIELD_WIDTH - WALL_CLEARANCE, FIELD_LENGTH - WALL_CLEARANCE)
runtime_start_idx = None
for i, (x, y) in enumerate(waypoints):
    if abs(x - runtime_start[0]) < 1e-3 and abs(y - runtime_start[1]) < 1e-3:
        runtime_start_idx = i
        break

if runtime_start_idx is not None:
    sx, sy = waypoints[runtime_start_idx]
    ax.plot(sx, sy, 'ms', markersize=16, label='Runtime Start (ToF Init)', zorder=8)
    ax.annotate('Runtime Start\n(ToF + IMU)',
                xy=(sx, sy), xytext=(sx - 15, sy - 10),
                textcoords='data', fontsize=10,
                arrowprops=dict(arrowstyle='->', color='magenta'),
                bbox=dict(boxstyle='round', facecolor='lavender', alpha=0.8))

# ========== 绘制所有路径点的碰撞圆（检查碰撞） ==========
# 每隔2个点画一个圆，避免太密集
for idx in range(0, len(waypoints), 2):
    circle = patches.Circle((waypoints[idx][0], waypoints[idx][1]),
                           ROBOT_RADIUS, linewidth=0.5,
                           edgecolor='orange', facecolor='yellow', alpha=0.15, zorder=3)
    ax.add_patch(circle)

# 在关键点（转弯处、基地绕行处）画实心圆
key_indices = [0, 10, 14, 20, 28, 32, 43, 47, 58, 62]  # 起点、四个角、基地绕行点
for idx in key_indices:
    if idx < len(waypoints):
        circle = patches.Circle((waypoints[idx][0], waypoints[idx][1]),
                               ROBOT_RADIUS, linewidth=1.5,
                               edgecolor='red', facecolor='none', linestyle='-', zorder=4)
        ax.add_patch(circle)

# 标注基地安全区域
ax.axhline(y=SOUTH_NEXUS_SAFE_Y, color='purple', linestyle=':', linewidth=2, alpha=0.5, label='Nexus Safe Zone')
ax.axhline(y=NORTH_NEXUS_SAFE_Y, color='purple', linestyle=':', linewidth=2, alpha=0.5)

# 设置坐标轴
ax.set_xlim(-8, FIELD_WIDTH + 8)
ax.set_ylim(-8, FIELD_LENGTH + 8)
ax.set_aspect('equal')
ax.set_xlabel('X (inches)', fontsize=14)
ax.set_ylabel('Y (inches)', fontsize=14)
ax.set_title('Wall-Following Path on ROBA 2025 Field\nRobot Center ~8.5" from Wall, Collision Radius 5.5"',
             fontsize=16, fontweight='bold')
ax.grid(True, alpha=0.3, linestyle='--')
ax.legend(loc='upper right', fontsize=12)

# 添加注释
info_text = f'Wall Clearance: {WALL_CLEARANCE}"\nCollision Radius: {ROBOT_RADIUS}"\nSafe Margin: {SAFE_MARGIN}"'
ax.text(5, 5, info_text, fontsize=11, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# 标注四个角
corners = [
    (WALL_CLEARANCE, WALL_CLEARANCE, 'Bottom-Left\nCorner'),
    (WALL_CLEARANCE, FIELD_LENGTH - WALL_CLEARANCE, 'Top-Left\nCorner'),
    (FIELD_WIDTH - WALL_CLEARANCE, FIELD_LENGTH - WALL_CLEARANCE, 'Top-Right\nCorner'),
    (FIELD_WIDTH - WALL_CLEARANCE, WALL_CLEARANCE, 'Bottom-Right\nCorner'),
]
for x, y, label in corners:
    ax.plot(x, y, 'k^', markersize=10, zorder=8)
    ax.text(x + 3, y + 3, label, fontsize=9, bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))

plt.tight_layout()
plt.savefig('wall_follow_path.png', dpi=200, bbox_inches='tight')
print("=" * 60)
print("路径图已保存到: wall_follow_path.png")
print("=" * 60)
print(f"路径点总数: {len(waypoints)}")
path_length = sum([np.sqrt((waypoints[i+1][0]-waypoints[i][0])**2 + (waypoints[i+1][1]-waypoints[i][1])**2) for i in range(len(waypoints)-1)])
print(f"路径总长度: {path_length:.1f} inches ({path_length/12:.1f} feet)")
print(f"车中心距墙: {WALL_CLEARANCE} inches")
print(f"碰撞半径: {ROBOT_RADIUS} inches")
print(f"安全余量: {SAFE_MARGIN} inches")
print("=" * 60)
plt.show()

