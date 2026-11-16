cmake_minimum_required(VERSION 3.8)
project(limo_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes worlds config launch
  DESTINATION share/${PROJECT_NAME}
)

# [GZ] steering dashboard (keyboard teleop window): ros2 run limo_gz dashboard_node.py
install(
  PROGRAMS scripts/dashboard_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
