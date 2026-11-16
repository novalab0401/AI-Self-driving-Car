"""LIMO(아커만)를 Gazebo Harmonic 에 올린다: 구동, 라이다, 깊이 카메라, IMU, 오도메트리, TF.
조종 창(dashboard)과 RViz 도 같이 띄운다.

  ros2 launch limo_gz sim.launch.py
  ros2 launch limo_gz sim.launch.py world:=empty.sdf
  ros2 launch limo_gz sim.launch.py dashboard:=false rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('limo_gz')
    xacro_file = os.path.join(pkg, 'urdf', 'limo_ackermann.urdf.xacro')
    bridge_file = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_file = os.path.join(pkg, 'rviz', 'gazebo.rviz')
    world = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'limo', '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}],
    )

    # [GZ] steering dashboard (keyboard teleop window), as in the original ackermann_sim.launch.py
    dashboard = Node(
        package='limo_gz',
        executable='dashboard_node.py',
        output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    # [GZ] RViz: grid, TF, laser scan, robot model, depth point cloud, camera image
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='worlds 폴더 안의 월드 파일 이름'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='조종 창(dashboard_node.py)을 같이 띄울지'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz 를 같이 띄울지'),
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        dashboard,
        rviz,
    ])
