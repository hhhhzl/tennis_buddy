# Tennis Buddy

## About


## Installation

#### Requirements: 
  - ROS2
  - Gazebo
  - Isaac Lab (optional for Nvidia GPU purpose)

We provide docker container integration of stacks above, please check:
[docker/readme.md](docker/README.md)

## Usage
Make sure the source is loaded:
```commandline
source /opt/ros/humble/setup.bash
```
Build
```commandline
cd tennisbuddy 
# make sure you are under ./tennis_buddy/tennisbuddy

colcon build
source install/setup.sh
```

## Run
```commandline
ros2 launch tennisbuddy_ros2_control miti_65.launch.py
```

## Simulation
Run in Gazebo:
```commandline
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf
```
![Running Tennisbuddy in Gazebo](./assets/gazebo_sim.png)


## Testing
```
ros2 launch tennisbuddy_gazebo miti_65_gazebo.launch.py world:=court.sdf world_name:=tennis_world use_sim_time:=true
ros2 launch tennisbuddy_perception ball_spawner.launch.py world:=tennis_world count:=15
ros2 launch tennisbuddy_perception perception_gazebo_gt.launch.py world:=tennis_world
ros2 launch tennisbuddy_planning navigation_launch.py use_sim_time:=true slam:=true map_file_name:=/home/autobots/tennis_buddy/tennisbuddy/ros2_control/maps/court_mapv2
ros2 launch tennisbuddy_planning planning_nav2_goal.launch.py use_sim_time:=true
```

# 
```
ros2 run tennisbuddy_planning nav2_goal_pusher \
    --ros-args \
    -p use_sim_time:=true \
    -p odom_topic:=/odometry/filtered \
    -p frame_id:=map \
    -p pickup_distance:=0.35
```

```
ros2 launch tennisbuddy_planning planning_nav2_goal.launch.py \
  use_sim_time:=true \
  publish_initial_pose:=True \
  initial_pose_x:=-5.0 \
  initial_pose_y:=0.0 \
  initial_pose_yaw:=0.0
```