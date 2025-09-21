cmake_minimum_required(VERSION 3.10)
project(control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
# Library
add_library(control_lib src/control_core.cpp)
target_include_directories(control_lib PUBLIC include)
ament_target_dependencies(control_lib rclcpp nav_msgs std_msgs geometry_msgs tf2)

# Executable
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp nav_msgs std_msgs geometry_msgs tf2)
target_link_libraries(control_node control_lib)

# Install
install(TARGETS
  control_node
  control_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()