#!/usr/bin/env python3
"""
Launch file to visualize the AMR Robot Model and TF tree in RViz2.
Starts:
1. robot_state_publisher (processes xacro into robot_description topic & broadcasts TF)
2. joint_state_publisher_gui (GUI sliders to spin wheels and test joint transforms)
3. rviz2 (3D visualizer pre-configured with RobotModel and TF)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 1. Package directory paths
    pkg_amr_description = get_package_share_directory('amr_description')

    xacro_file = os.path.join(pkg_amr_description, 'urdf', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_amr_description, 'rviz', 'view_robot.rviz')

    # 2. Convert Xacro into URDF XML using the 'xacro' CLI command
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 3. Robot State Publisher Node
    # Publishes static and joint-state TF transforms and the /robot_description topic
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    # 4. Joint State Publisher GUI Node
    # Provides an interactive slider GUI to actuate continuous wheel joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 5. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
