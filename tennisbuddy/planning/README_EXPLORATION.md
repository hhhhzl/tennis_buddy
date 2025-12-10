# 探索功能使用说明

## 概述

这个探索功能允许机器人在没有看到球时自主探索环境，当检测到球时自动切换到捡球模式。

## 架构

系统包含两个主要节点：

1. **exploration_waypoint_manager**: 生成和管理探索用的 waypoint 序列
   - 支持预定义 waypoint 列表
   - 支持基于网格的自动生成

2. **exploration_ball_collector**: 整合探索和捡球逻辑
   - 状态机：`EXPLORING` / `COLLECTING_BALL`
   - 使用 Nav2 的 `NavigateThroughPoses` action 进行探索
   - 使用 Nav2 的 `NavigateToPose` action 进行捡球

## 使用方法

### 1. 编译包

```bash
cd /path/to/tennisbuddy
colcon build --packages-select tennisbuddy_planning
source install/setup.bash
```

### 2. 配置参数

编辑 `tennisbuddy/planning/configs/exploration_params.yaml`：

- **探索区域**: 设置 `exploration_area_min_x/max_x/min_y/max_y`
- **Waypoint 间距**: 调整 `waypoint_spacing`（米）
- **预定义 waypoint**: 设置 `use_predefined_waypoints: true` 并添加坐标列表

### 3. 启动系统

```bash
ros2 launch tennisbuddy_planning planning_nav2_exploration.launch.py
```

或者带参数：

```bash
ros2 launch tennisbuddy_planning planning_nav2_exploration.launch.py \
    use_sim_time:=true \
    slam:=True \
    publish_initial_pose:=True \
    initial_pose_x:=-5.0 \
    initial_pose_y:=0.0 \
    initial_pose_yaw:=0.0
```

## 工作流程

1. **启动后**: 
   - `exploration_waypoint_manager` 生成 waypoint 序列并发布到 `/exploration_waypoints`
   - `exploration_ball_collector` 订阅 waypoint 和球位置

2. **无球时**:
   - 状态：`EXPLORING`
   - 行为：按 waypoint 序列移动，使用 `NavigateThroughPoses`

3. **检测到球时**:
   - 状态：切换到 `COLLECTING_BALL`
   - 行为：取消当前探索任务，发送球位置目标，使用 `NavigateToPose`

4. **球消失后**:
   - 等待 `ball_detection_timeout` 秒
   - 状态：切换回 `EXPLORING`
   - 行为：继续探索未访问的 waypoint

## 话题

- `/ball_positions` (PoseArray): 球的位置（来自 perception）
- `/exploration_waypoints` (PoseArray): 探索 waypoint 序列
- `/odometry/filtered` (Odometry): 机器人位置

## Action 服务

- `navigate_through_poses` (NavigateThroughPoses): 探索导航
- `navigate_to_pose` (NavigateToPose): 捡球导航

## 参数说明

### exploration_waypoint_manager

- `waypoint_spacing`: waypoint 之间的间距（米）
- `exploration_area_*`: 探索区域的边界
- `use_predefined_waypoints`: 是否使用预定义 waypoint
- `predefined_waypoints`: 预定义 waypoint 列表 `[x, y, yaw]`

### exploration_ball_collector

- `pickup_distance`: 捡球时的停止距离（米）
- `ball_detection_timeout`: 球检测超时时间（秒）
- `waypoint_visit_threshold`: 判断 waypoint 已访问的距离阈值（米）
- `use_navigate_through_poses`: 是否使用 `NavigateThroughPoses`（推荐 true）

## 注意事项

1. 确保 Nav2 已正确配置并运行
2. 确保 `navigate_through_poses` action server 可用（在 nav2_params.yaml 中配置）
3. 根据实际地图大小调整探索区域参数
4. 如果使用预定义 waypoint，确保坐标在有效区域内

## 与现有系统的关系

- **不修改** `nav2_goal_pusher.py` - 原有功能保持不变
- **新增** 探索功能，可以独立使用
- 可以与 `planning_nav2_goal.launch.py` 并行运行（使用不同的命名空间）

