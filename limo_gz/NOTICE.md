# limo_gz

The robot model in `urdf/` and `meshes/` comes from the `limo_car` package of
[agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) (declared license: Apache-2.0),
through the fork [JACKSON-KIM-JAEHO/limo_ros2](https://github.com/JACKSON-KIM-JAEHO/limo_ros2), branch `humble_low_Mesh`.

Changes made here (marked `[GZ]` in the files):

- Gazebo Classic drive plugins replaced with the Gazebo Harmonic Ackermann steering and joint state publisher systems.
- Lidar sensor changed from `ray` + `libgazebo_ros_ray_sensor.so` to `gpu_lidar`.
- Mesh paths changed to this package.
- `scripts/dashboard_node.py` comes from the `limo_dashboard` package of the same fork and branch.
  It publishes `/cmd_vel` only while the teleop toggle is on, drives only while a key is held,
  and keeps the same wheel direction when reversing.
- `models/traffic_light` comes from `limo_car/models/traffic_light` of the same fork and branch,
  with the Gazebo Classic material scripts replaced by plain colors.
- `src/traffic_light_plugin.cpp` is written new for Gazebo Harmonic. It keeps the topic and color
  names of the Classic `limo_plugin` but none of its code.
- `rviz/gazebo.rviz` comes from `limo_car/rviz` of the same fork and branch, unchanged.
- Depth camera and IMU use Gazebo Harmonic sensors, bridged to ROS in `config/bridge.yaml`.
