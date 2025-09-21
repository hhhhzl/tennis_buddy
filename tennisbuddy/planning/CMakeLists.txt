cmake_minimum_required(VERSION 3.10)
project(tennisbuddy_planning)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  configs
  resource
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
