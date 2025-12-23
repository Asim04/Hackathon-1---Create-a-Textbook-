#!/bin/bash
# Launch Script for URDF Visualization
# Purpose: Start robot_state_publisher, joint_state_publisher_gui, and RViz2
# Usage: ./launch_urdf_rviz.sh

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
URDF_FILE="$SCRIPT_DIR/simple_humanoid_arm.urdf"

echo "==================================="
echo "URDF Visualization Launcher"
echo "==================================="
echo "URDF File: $URDF_FILE"
echo ""

# Check if URDF file exists
if [ ! -f "$URDF_FILE" ]; then
    echo "ERROR: URDF file not found: $URDF_FILE"
    exit 1
fi

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS 2 Humble environment sourced"
else
    echo "ERROR: ROS 2 Humble not found. Please install ROS 2 Humble."
    exit 1
fi

echo ""
echo "Starting ROS 2 nodes..."
echo "Press Ctrl+C to stop all nodes"
echo ""

# Launch robot_state_publisher in background
echo "[1/3] Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat $URDF_FILE)" &
RSP_PID=$!
sleep 2

# Launch joint_state_publisher_gui in background
echo "[2/3] Starting joint_state_publisher_gui..."
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
JSP_PID=$!
sleep 2

# Launch RViz2 (foreground)
echo "[3/3] Starting RViz2..."
echo ""
echo "RViz2 Configuration:"
echo "  1. Click 'Add' button"
echo "  2. Select 'RobotModel'"
echo "  3. Set Fixed Frame to 'base_link'"
echo "  4. Use Joint State Publisher GUI sliders to move joints"
echo ""
rviz2 &
RVIZ_PID=$!

# Wait for RViz to close, then clean up
wait $RVIZ_PID

echo ""
echo "Shutting down..."
kill $RSP_PID $JSP_PID 2>/dev/null
echo "Done."
