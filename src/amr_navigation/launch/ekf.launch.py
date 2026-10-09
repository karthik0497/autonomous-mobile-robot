#!/usr/bin/env python3
"""
Launch file for Extended Kalman Filter (EKF) state estimation.
Uses robot_localization package to fuse wheel odometry and IMU.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    ekf_config_path = os.path.join(pkg_amr_navigation, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # EKF Filter Node from robot_localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        ekf_node
    ])
