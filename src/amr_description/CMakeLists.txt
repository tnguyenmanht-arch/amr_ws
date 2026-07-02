cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

# Script Python chạy được qua `ros2 run amr_description teleop_relay.py`
install(PROGRAMS
  scripts/teleop_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
