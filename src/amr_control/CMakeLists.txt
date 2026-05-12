cmake_minimum_required(VERSION 3.8)
project(amr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ackermann_msgs là optional — comment out nếu chưa cài
# find_package(ackermann_msgs REQUIRED)

include_directories(include)

add_executable(ackermann_controller_node
  src/ackermann_controller_node.cpp
)
ament_target_dependencies(ackermann_controller_node
  rclcpp std_msgs geometry_msgs nav_msgs
)

install(TARGETS ackermann_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
