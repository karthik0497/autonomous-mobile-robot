#!/usr/bin/env python3
"""
Gazebo Harmonic Simulation Launch File for AMR Robot.
Launches:
1. Gazebo Harmonic simulator (gz sim) with obstacles.world
2. robot_state_publisher (with use_sim_time=True)
3. Spawner entity (ros_gz_sim create)
4. ros_gz_bridge (parameter_bridge for /cmd_vel, /odom, /clock, /joint_states, /tf)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    xacro_file = os.path.join(pkg_amr_description, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_amr_gazebo, 'worlds', 'obstacles.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Convert Xacro to URDF string
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1. Start Gazebo Sim with custom world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 1 {world_file}'}.items()
    )

    # 2. Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # 3. Spawn Robot in Gazebo from robot_description topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'amr_robot',
            '-z', '0.05'
        ]
    )

    # 4. ROS 2 <-> Gazebo Bridge (parameter_bridge)
    # Bridges topics between ROS 2 DDS and Gazebo Harmonic transport
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Clock (Gazebo -> ROS)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Velocity Command (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Joint States (Gazebo -> ROS)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # 2D LiDAR (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # RGB Camera Image & Info (Gazebo -> ROS)
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # IMU Data (Gazebo -> ROS)
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge
    ])
