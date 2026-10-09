import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_amr_navigation = get_package_share_directory('amr_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz visualization if true'
    )

    # 1. Gazebo Harmonic Simulation with Warehouse Arena
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_gazebo, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Extended Kalman Filter (EKF Sensor Fusion) - Starts at t=3s
    ekf_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_amr_navigation, 'launch', 'ekf.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # 3. SLAM Toolbox & RViz2 - Starts at t=6s
    slam_cmd = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_amr_navigation, 'launch', 'slam.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz': use_rviz
                }.items()
            )
        ]
    )

    # 4. Autonomous Warehouse Loop Navigator Node - Starts at t=8s
    navigator_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='amr_navigation',
                executable='loop_navigator.py',
                name='warehouse_loop_navigator',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        sim_cmd,
        ekf_cmd,
        slam_cmd,
        navigator_node
    ])
