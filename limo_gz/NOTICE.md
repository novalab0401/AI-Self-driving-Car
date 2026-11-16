# limo_gz

The robot model in `urdf/` and `meshes/` comes from the `limo_car` package of
[agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) (declared license: Apache-2.0),
through the fork [JACKSON-KIM-JAEHO/limo_ros2](https://github.com/JACKSON-KIM-JAEHO/limo_ros2), branch `humble_low_Mesh`.

Changes made here (marked `[GZ]` in the files):

- Gazebo Classic drive plugins replaced with the Gazebo Harmonic Ackermann steering and joint state publisher systems.
- Lidar sensor changed from `ray` + `libgazebo_ros_ray_sensor.so` to `gpu_lidar`.
- Mesh paths changed to this package.
- Depth camera and IMU are not attached yet.
- `scripts/dashboard_node.py` comes from the `limo_dashboard` package of the same fork and branch.
  Only the steering panel is ported; it publishes `/cmd_vel` only while the teleop toggle is on.
