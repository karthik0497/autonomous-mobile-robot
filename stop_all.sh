#!/usr/bin/env bash
# ==============================================================================
# Autonomous Mobile Robot - Stop All Simulation & Navigation Processes
# ==============================================================================

echo "🛑 Stopping all AMR simulation, navigation, and Gazebo processes..."

# Kill specific ROS 2 nodes and Gazebo Harmonic processes
pkill -f "loop_navigator.py" 2>/dev/null && echo "  - Stopped loop_navigator.py"
pkill -f "teleop_twist_keyboard" 2>/dev/null && echo "  - Stopped teleop_twist_keyboard"
pkill -f "async_slam_toolbox_node" 2>/dev/null && echo "  - Stopped slam_toolbox"
pkill -f "ekf_node" 2>/dev/null && echo "  - Stopped ekf_node"
pkill -f "rviz2" 2>/dev/null && echo "  - Stopped rviz2"
pkill -f "robot_state_publisher" 2>/dev/null && echo "  - Stopped robot_state_publisher"
pkill -f "parameter_bridge" 2>/dev/null && echo "  - Stopped ros_gz_bridge"
pkill -f "gz sim" 2>/dev/null && echo "  - Stopped Gazebo Sim"

sleep 1

# Check if any lingering gz processes remain
if pgrep -f "gz sim" > /dev/null; then
    echo "  - Force killing remaining Gazebo processes..."
    pkill -9 -f "gz sim" 2>/dev/null
fi

echo "✅ All processes cleanly stopped! Environment is clear."
