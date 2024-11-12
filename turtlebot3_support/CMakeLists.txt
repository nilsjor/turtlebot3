cmake_minimum_required(VERSION 3.5)
project(turtlebot3_support)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(teleop_twist_joy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
)

# Add the executable for tf2_transform_node
add_executable(tf2_transform_node src/tf2_transform_node.cpp)
ament_target_dependencies(tf2_transform_node ${dependencies})

install(TARGETS tf2_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

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
