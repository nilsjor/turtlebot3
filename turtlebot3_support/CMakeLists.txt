cmake_minimum_required(VERSION 3.5)
project(turtlebot3_support)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS script/ping_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
