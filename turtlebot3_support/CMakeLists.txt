cmake_minimum_required(VERSION 3.5)
project(turtlebot3_support)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

file(
  READ ${CMAKE_CURRENT_SOURCE_DIR}/resource/bash_aliases_template TEXT
)
  
if(NOT DEFINED ENV{ROS_WS_PATH})
  set(ROS_WS_PATH "$ENV{HOME}/ros2_ws")
else()
  set(ROS_WS_PATH "$ENV{ROS_WS_PATH}")
endif()

if(NOT DEFINED ENV{CYCLONEDDS_URI})
  set(CYCLONEDDS_URI "$ENV{HOME}/husarnet-cyclonedds.xml")
else()
  set(CYCLONEDDS_URI "$ENV{CYCLONEDDS_URI}")
endif()

if(NOT DEFINED ENV{FASTRTPS_DEFAULT_PROFILES_FILE})
  set(FASTRTPS_DEFAULT_PROFILES_FILE "$ENV{HOME}/husarnet-fastdds-simple.xml")
else()
  set(FASTRTPS_DEFAULT_PROFILES_FILE "$ENV{FASTRTPS_DEFAULT_PROFILES_FILE}")
endif()

if(NOT DEFINED ENV{RMW_IMPLEMENTATION})
  set(RMW_IMPLEMENTATION "rmw_fastrtps_cpp")
else()
  set(RMW_IMPLEMENTATION "$ENV{RMW_IMPLEMENTATION}")
endif()

string(
  REPLACE
  "@ROS_WS_PATH@" ${ROS_WS_PATH}
  TEXT "${TEXT}"
)
    
string(
  REPLACE
  "@CYCLONEDDS_URI@" ${CYCLONEDDS_URI}
  TEXT "${TEXT}"
) 

string(
  REPLACE
  "@FASTRTPS_DEFAULT_PROFILES_FILE@" ${FASTRTPS_DEFAULT_PROFILES_FILE}
  TEXT "${TEXT}"
)

string(
  REPLACE
  "@RMW_IMPLEMENTATION@" ${RMW_IMPLEMENTATION}
  TEXT "${TEXT}"
)

file(
  GENERATE 
  OUTPUT $ENV{HOME}/.bash_aliases 
  CONTENT "${TEXT}"
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS script/ping_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
