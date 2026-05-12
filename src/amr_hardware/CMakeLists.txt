cmake_minimum_required(VERSION 3.8)
project(amr_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# Node giao tiếp serial với STM32
add_executable(serial_driver_node
  src/serial_driver_node.cpp
  src/stm32_comm.cpp
)
ament_target_dependencies(serial_driver_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros
)

install(TARGETS serial_driver_node
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
