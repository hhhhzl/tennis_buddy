cmake_minimum_required(VERSION 3.10)
project(tennisbuddy_perception)

find_package(ament_cmake REQUIRED)

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

ament_python_install_package(${CMAKE_CURRENT_SOURCE_DIR})

ament_package()