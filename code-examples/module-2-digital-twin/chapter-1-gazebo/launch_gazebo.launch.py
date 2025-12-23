#!/usr/bin/env python3
"""
ROS 2 Launch File for Chapter 1: Gazebo Physics Simulation
Purpose: Launch Gazebo with custom world and spawn humanoid robot
Compatible: ROS 2 Humble, Gazebo Classic 11
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get path to this package's share directory
    # Note: This assumes you've installed this as a ROS 2 package
    # For standalone use, replace with absolute path to world file
    pkg_share = FindPackageShare('module_2_gazebo_examples').find('module_2_gazebo_examples')
    world_file = os.path.join(pkg_share, 'worlds', 'humanoid_physics.world')

    # Alternative: Use absolute path for standalone execution
    # world_file = '/path/to/code-examples/module-2-digital-twin/chapter-1-gazebo/humanoid_physics.world'

    # Launch Gazebo with custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'pause': 'false',  # Start simulation immediately
        }.items()
    )

    # Spawn humanoid robot from URDF
    # Note: Update path to your humanoid.urdf from Module 1
    robot_urdf_path = os.path.join(pkg_share, 'urdf', 'humanoid.urdf')

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_robot',
            '-file', robot_urdf_path,
            '-x', '0',
            '-y', '0',
            '-z', '1.0',  # Spawn 1 meter above ground (allows falling demonstration)
        ],
        output='screen'
    )

    # Publish joint states (for /joint_states topic)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Robot state publisher (broadcasts TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(robot_urdf_path).read()}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_robot,
        joint_state_publisher,
        robot_state_publisher,
    ])

# Usage:
# 1. Copy this file to a ROS 2 package (e.g., module_2_gazebo_examples/launch/)
# 2. Update paths to world_file and robot_urdf_path
# 3. Build package: colcon build --packages-select module_2_gazebo_examples
# 4. Source workspace: source install/setup.bash
# 5. Launch: ros2 launch module_2_gazebo_examples launch_gazebo.launch.py
#
# Expected Output:
# - Gazebo GUI opens with custom world
# - Humanoid robot spawns at (0, 0, 1.0)
# - Robot falls due to gravity, collides with ground
# - /joint_states topic publishes robot joint positions
# - Real-time factor displayed in Gazebo GUI (target: 1.0)
