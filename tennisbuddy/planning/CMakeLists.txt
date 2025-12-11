cmake_minimum_required(VERSION 3.10)
project(tennisbuddy_planning)

find_package(ament_cmake REQUIRED)

# Install executable wrapper scripts (these will be in libexec/tennisbuddy_perception/)
install(PROGRAMS
  scripts/nav2_goal_pusher
  scripts/nav2_path_bridge
  scripts/exploration_ball_collector
  scripts/exploration_waypoint_manager
  scripts/planner_node
  scripts/traj_tracker
  scripts/publish_initial_pose
  DESTINATION lib/${PROJECT_NAME}
)

# Install source Python files so scripts can import them
install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
  PATTERN "__pycache__" EXCLUDE
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(DIRECTORY
  launch
  configs
  resource
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()