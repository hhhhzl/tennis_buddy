cmake_minimum_required(VERSION 3.10)
project(tennisbuddy_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/nodes/ball_spawner.py
  src/nodes/ball_groundtruth.py
  src/nodes/mock_spawn_service.py
  DESTINATION lib/${PROJECT_NAME}
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