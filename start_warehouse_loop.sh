#!/usr/bin/env bash
# ==============================================================================
# Autonomous Mobile Robot - Unified Startup Script
# Automatically launches Gazebo, EKF, SLAM, and Autonomous Loop Navigator
# in organized, dedicated terminal tabs.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODE="autonomous"
if [[ "$1" == "--teleop" ]]; then
    MODE="teleop"
fi

echo "========================================================================"
echo "🤖 Starting Autonomous Mobile Robot (AMR) System"
echo "   Directory: $SCRIPT_DIR"
echo "   Mode:      $MODE"
echo "========================================================================"

# 1. Clean up any stale running instances
echo "🧹 Cleaning up any previous running processes..."
./stop_all.sh > /dev/null 2>&1 || true

# 2. Verify workspace build
echo "⚙️ Building/updating workspace with colcon..."
colcon build --symlink-install

# 3. Check for graphical display and gnome-terminal
if [[ -n "$DISPLAY" ]] && command -v gnome-terminal &> /dev/null; then
    echo "🖥️ Launching 4 tabs in GNOME Terminal window..."

    # Tab 1: Gazebo Sim (creates new window)
    gnome-terminal --window --title="1. Gazebo Sim" --working-directory="$SCRIPT_DIR" -- bash -c "
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
      echo '=== Terminal 1: Gazebo Harmonic Simulation ==='
      ros2 launch amr_gazebo sim.launch.py
      exec bash
    "

    sleep 1

    # Tab 2: EKF Fusion (adds tab to existing window)
    gnome-terminal --tab --title="2. EKF Fusion" --working-directory="$SCRIPT_DIR" -- bash -c "
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
      echo 'Waiting 3 seconds for Gazebo to stabilize...'
      sleep 3
      echo '=== Terminal 2: Robot Localization (EKF) ==='
      ros2 launch amr_navigation ekf.launch.py
      exec bash
    "

    sleep 1

    # Tab 3: SLAM & RViz (adds tab to existing window)
    gnome-terminal --tab --title="3. SLAM & RViz" --working-directory="$SCRIPT_DIR" -- bash -c "
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
      echo 'Waiting 5 seconds for EKF odometry...'
      sleep 5
      echo '=== Terminal 3: SLAM Toolbox & RViz2 ==='
      ros2 launch amr_navigation slam.launch.py
      exec bash
    "

    sleep 1

    # Tab 4: Autonomous Loop Navigator or Teleop
    if [[ "$MODE" == "teleop" ]]; then
        gnome-terminal --tab --title="4. Teleop Keyboard" --working-directory="$SCRIPT_DIR" -- bash -c "
          source /opt/ros/jazzy/setup.bash
          source install/setup.bash
          echo 'Waiting 7 seconds for SLAM map...'
          sleep 7
          echo '=== Terminal 4: Teleop Keyboard ==='
          echo 'Focus this window and use i/j/k/l to drive!'
          ros2 run teleop_twist_keyboard teleop_twist_keyboard
          exec bash
        "
    else
        gnome-terminal --tab --title="4. Loop Navigator" --working-directory="$SCRIPT_DIR" -- bash -c "
          source /opt/ros/jazzy/setup.bash
          source install/setup.bash
          echo 'Waiting 7 seconds for sensors to stream...'
          sleep 7
          echo '=== Terminal 4: Autonomous Warehouse Loop Navigator ==='
          ros2 run amr_navigation loop_navigator.py
          exec bash
        "
    fi

    echo ""
    echo "✅ Successfully launched all 4 tabs in GNOME Terminal!"
    echo "   - Tab 1: Gazebo Harmonic (Warehouse Arena with Table, Chairs, Racks)"
    echo "   - Tab 2: EKF Odometry Fusion"
    echo "   - Tab 3: SLAM Toolbox & RViz2"
    echo "   - Tab 4: Autonomous Loop Navigator"
    echo ""
    echo "💡 To stop everything at any time, run: ./stop_all.sh"

else
    echo "🖥️ Running master ROS 2 launch file in current terminal..."
    echo "Press Ctrl+C to stop all nodes."

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

    if [[ "$MODE" == "teleop" ]]; then
        ros2 launch amr_navigation warehouse_loop.launch.py &
        sleep 8
        ros2 run teleop_twist_keyboard teleop_twist_keyboard
    else
        ros2 launch amr_navigation warehouse_loop.launch.py
    fi
fi
