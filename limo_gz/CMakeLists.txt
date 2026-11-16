cmake_minimum_required(VERSION 3.8)
project(limo_gz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# [GZ] With a conda environment active (even just "base"), CMake finds conda's
# Python and Protobuf first. Those do not match the ROS 2 / Gazebo libraries from apt
# and the build fails, so keep CMake out of the conda prefix.
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  list(APPEND CMAKE_IGNORE_PATH "$ENV{CONDA_PREFIX}/bin")
  set(Python3_FIND_VIRTUALENV STANDARD)
endif()

find_package(ament_cmake REQUIRED)

# [GZ] Gazebo Harmonic libraries, as packaged for ROS 2 Jazzy (the *_vendor packages
# make the gz-* CMake packages findable).
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)

# [GZ] traffic light color plugin (was: limo_plugin/traffic_light_plugin for Gazebo Classic)
add_library(traffic_light_plugin SHARED src/traffic_light_plugin.cpp)
target_link_libraries(traffic_light_plugin PRIVATE
  gz-plugin2::register
  gz-sim8::core
)
install(TARGETS traffic_light_plugin DESTINATION lib)

install(
  DIRECTORY urdf meshes worlds config launch models rviz
  DESTINATION share/${PROJECT_NAME}
)

# [GZ] steering dashboard (keyboard teleop window): ros2 run limo_gz dashboard_node.py
install(
  PROGRAMS scripts/dashboard_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# [GZ] lets Gazebo find the plugin (lib/) and model://traffic_light (models/)
# after `source install/setup.bash`
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
