---
sidebar_position: 5
title: "Chapter 4: Lab - Building a ROS 2 Robot"
description: "Hands-on lab integrating ROS 2 nodes, Python agents, and URDF models into a complete humanoid robot system"
---

# Chapter 4: Hands-On Lab - Building a ROS 2 Robot

**Module**: The Robotic Nervous System (ROS 2)
**Estimated Time**: 3-4 hours
**Prerequisites**: Chapters 1-3 completed, Ubuntu 22.04, ROS 2 Humble installed

---

## Learning Objectives

By the end of this lab, you will be able to:
- Integrate ROS 2 communication, Python AI agents, and URDF models into a unified robot system
- Design and implement a complete sensor-to-actuator control pipeline
- Debug multi-component ROS 2 systems using systematic troubleshooting approaches
- Demonstrate autonomous robot behavior through simulation

---

## Lab Overview

This hands-on lab synthesizes concepts from Chapters 1-3 into a complete autonomous humanoid robot system. You will build:

1. **URDF Robot Model**: 6-DOF humanoid with torso, arms, and head
2. **Python Perception Agent**: Processes simulated sensor data (vision, distance)
3. **Python Control Agent**: Sends joint commands based on perception
4. **ROS 2 Integration**: Topics and nodes coordinating perception, decision-making, and control

**System Architecture**:
```
┌─────────────────────────────────────────────────────────┐
│               ROBOT HARDWARE (Simulated)                │
│  - URDF Model (6-DOF humanoid)                          │
│  - Distance Sensors (left/right arms)                   │
│  - Joint Controllers (shoulder, elbow)                  │
└──────────────────┬──────────────────┬───────────────────┘
                   │                  │
        /sensor_data topic    /joint_commands topic
                   │                  │
┌──────────────────▼──────────────────┴───────────────────┐
│             ROS 2 MIDDLEWARE (DDS)                      │
└──────────────────┬──────────────────┬───────────────────┘
                   │                  │
        ┌──────────▼──────────┐  ┌───▼────────────────┐
        │ PERCEPTION AGENT     │  │  CONTROL AGENT     │
        │ (Python rclpy)       │  │  (Python rclpy)    │
        │                      │  │                    │
        │ - Subscribe: sensors │  │ - Subscribe: goals │
        │ - Publish: detections│  │ - Publish: commands│
        └──────────────────────┘  └────────────────────┘
```

---

## Part 1: Setting Up the Robot Model (45 minutes)

### Step 1.1: Create 6-DOF Humanoid URDF

Create a file `lab_humanoid.urdf` with complete upper-body humanoid:

**Robot Specifications**:
- **Torso**: 0.4m height, 0.3m width, 0.2m depth (10kg)
- **Head**: Sphere radius 0.1m (0.5kg)
- **Arms** (left and right):
  - Upper arm: Cylinder length 0.3m, radius 0.04m (1kg each)
  - Forearm: Cylinder length 0.25m, radius 0.03m (0.5kg each)

**Joints**:
1. `neck`: Revolute, head rotation (±90°)
2. `right_shoulder_pitch`: Revolute, right arm raise (±90°)
3. `right_elbow`: Revolute, right arm bend (0° to 135°)
4. `left_shoulder_pitch`: Revolute, left arm raise (±90°)
5. `left_elbow`: Revolute, left arm bend (0° to 135°)

### Implementation

<details>
<summary>Click to reveal URDF starter template (try creating it yourself first!)</summary>

```xml
<?xml version="1.0"?>
<robot name="lab_humanoid">

  <!-- Base Link -->
  <link name="base_link"/>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.3 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.3 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.17" iyy="0.15" izz="0.11" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
  </joint>

  <!-- TODO: Add head, arms (left/right), joints -->
  <!-- See Chapter 3 for joint/link examples -->

</robot>
```
</details>

**✅ Checkpoint 1.1**: Verify URDF with:
```bash
ros2 run urdf_tutorial check_urdf lab_humanoid.urdf
# Should show: Successfully Parsed XML
```

---

### Step 1.2: Visualize in RViz

Launch the robot model:

```bash
# Terminal 1
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat lab_humanoid.urdf)"

# Terminal 2
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3
rviz2
```

**Configure RViz**:
1. Add → RobotModel
2. Fixed Frame → `base_link`
3. Verify all links visible (torso, head, 2 arms)

**✅ Checkpoint 1.2**: Move joint sliders and confirm arms/head move correctly.

---

## Part 2: Creating the Perception Agent (60 minutes)

### Step 2.1: Design the Perception Pipeline

**Goal**: Subscribe to distance sensors, detect obstacles, publish detection messages.

**Topics**:
- **Subscribe**: `/left_distance` (std_msgs/Float32), `/right_distance` (std_msgs/Float32)
- **Publish**: `/obstacle_detected` (std_msgs/String) - Contains "left", "right", "both", or "none"

### Step 2.2: Implement Perception Agent

Create `perception_agent.py`:

```python
#!/usr/bin/env python3
"""
Lab Perception Agent
Purpose: Process left/right distance sensors and publish obstacle detection messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class PerceptionAgent(Node):
    """
    Perception agent for humanoid robot obstacle detection.

    Subscribes to:
        /left_distance (Float32): Distance from left arm sensor
        /right_distance (Float32): Distance from right arm sensor

    Publishes to:
        /obstacle_detected (String): "left", "right", "both", "none"
    """

    def __init__(self):
        super().__init__('perception_agent')

        # Subscribers for left/right distance sensors
        self.left_sub = self.create_subscription(
            Float32, '/left_distance', self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Float32, '/right_distance', self.right_callback, 10)

        # Publisher for obstacle detection
        self.detection_pub = self.create_publisher(String, '/obstacle_detected', 10)

        # Sensor state
        self.left_distance = None
        self.right_distance = None
        self.detection_threshold = 0.5  # meters

        # Timer for periodic detection processing
        self.timer = self.create_timer(0.1, self.process_detection)  # 10 Hz

        self.get_logger().info('Perception Agent started')
        self.get_logger().info(f'Detection threshold: {self.detection_threshold}m')

    def left_callback(self, msg):
        """Update left sensor reading"""
        self.left_distance = msg.data

    def right_callback(self, msg):
        """Update right sensor reading"""
        self.right_distance = msg.data

    def process_detection(self):
        """
        Process sensor data and publish detection message.
        Called at 10 Hz by timer.
        """
        if self.left_distance is None or self.right_distance is None:
            # Waiting for sensor data
            return

        # Detect obstacles based on threshold
        left_obstacle = self.left_distance < self.detection_threshold
        right_obstacle = self.right_distance < self.detection_threshold

        # Determine detection message
        if left_obstacle and right_obstacle:
            detection = "both"
        elif left_obstacle:
            detection = "left"
        elif right_obstacle:
            detection = "right"
        else:
            detection = "none"

        # Publish detection
        msg = String()
        msg.data = detection
        self.detection_pub.publish(msg)

        # Log detection (only when obstacles detected)
        if detection != "none":
            self.get_logger().info(
                f'Obstacle detected: {detection} '
                f'(L: {self.left_distance:.2f}m, R: {self.right_distance:.2f}m)'
            )


def main(args=None):
    rclpy.init(args=args)
    agent = PerceptionAgent()
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**✅ Checkpoint 2.1**: Test perception agent:
```bash
# Terminal 1: Run agent
python3 perception_agent.py

# Terminal 2: Simulate left obstacle
ros2 topic pub /left_distance std_msgs/msg/Float32 "data: 0.3"

# Terminal 3: Simulate right obstacle
ros2 topic pub /right_distance std_msgs/msg/Float32 "data: 0.3"

# Terminal 4: Monitor detection output
ros2 topic echo /obstacle_detected
# Should show: data: 'both'
```

---

## Part 3: Creating the Control Agent (60 minutes)

### Step 3.1: Design Control Logic

**Goal**: Subscribe to obstacle detections, command arm movements to avoid obstacles.

**Behavior**:
- **No obstacle**: Arms at neutral position (0° shoulder, 90° elbow)
- **Left obstacle**: Raise left arm (shoulder -45°)
- **Right obstacle**: Raise right arm (shoulder 45°)
- **Both obstacles**: Raise both arms

**Topics**:
- **Subscribe**: `/obstacle_detected` (String)
- **Publish**: `/joint_commands` (sensor_msgs/JointState)

### Step 3.2: Implement Control Agent

Create `control_agent.py`:

```python
#!/usr/bin/env python3
"""
Lab Control Agent
Purpose: Command arm movements based on obstacle detection
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math


class ControlAgent(Node):
    """
    Control agent for humanoid arm control based on obstacle detection.

    Subscribes to:
        /obstacle_detected (String): Detection from perception agent

    Publishes to:
        /joint_commands (JointState): Joint position commands
    """

    def __init__(self):
        super().__init__('control_agent')

        # Subscriber for obstacle detection
        self.detection_sub = self.create_subscription(
            String, '/obstacle_detected', self.detection_callback, 10)

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Joint position targets (radians)
        self.neutral_shoulder = 0.0  # Arms at side
        self.raised_shoulder = math.radians(-45)  # Arms raised
        self.elbow_angle = math.radians(90)  # Elbow bent 90°

        self.get_logger().info('Control Agent started')

    def detection_callback(self, msg):
        """
        Process obstacle detection and command arm movements.

        Args:
            msg (String): Detection message ("left", "right", "both", "none")
        """
        detection = msg.data

        # Determine target joint positions
        if detection == "left":
            left_shoulder = self.raised_shoulder
            right_shoulder = self.neutral_shoulder
        elif detection == "right":
            left_shoulder = self.neutral_shoulder
            right_shoulder = self.raised_shoulder
        elif detection == "both":
            left_shoulder = self.raised_shoulder
            right_shoulder = self.raised_shoulder
        else:  # "none"
            left_shoulder = self.neutral_shoulder
            right_shoulder = self.neutral_shoulder

        # Create JointState message
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = [
            'left_shoulder_pitch',
            'left_elbow',
            'right_shoulder_pitch',
            'right_elbow'
        ]
        joint_cmd.position = [
            left_shoulder,
            self.elbow_angle,
            right_shoulder,
            self.elbow_angle
        ]

        # Publish command
        self.joint_pub.publish(joint_cmd)

        # Log command
        self.get_logger().info(
            f'Command: {detection} → '
            f'L_shoulder={math.degrees(left_shoulder):.1f}°, '
            f'R_shoulder={math.degrees(right_shoulder):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    agent = ControlAgent()
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**✅ Checkpoint 3.1**: Test control agent:
```bash
# Terminal 1: Run control agent
python3 control_agent.py

# Terminal 2: Simulate detection
ros2 topic pub /obstacle_detected std_msgs/msg/String "data: 'left'"

# Terminal 3: Monitor joint commands
ros2 topic echo /joint_commands
# Should show left_shoulder_pitch at -0.785 rad (-45°)
```

---

## Part 4: System Integration (45 minutes)

### Step 4.1: Create Launch File

Create `lab_system_launch.py` to start all components:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os

def generate_launch_description():
    # Path to URDF file
    urdf_file = os.path.join(
        os.getcwd(), 'lab_humanoid.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher (reads /joint_commands, publishes /joint_states)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'source_list': ['/joint_commands']}]
        ),

        # Perception Agent
        Node(
            package='lab_humanoid',  # Replace with your package name
            executable='perception_agent.py',
            name='perception_agent'
        ),

        # Control Agent
        Node(
            package='lab_humanoid',
            executable='control_agent.py',
            name='control_agent'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
```

**✅ Checkpoint 4.1**: Launch full system:
```bash
ros2 launch lab_system_launch.py
# All nodes should start, RViz opens
```

---

### Step 4.2: Test Complete System

**Test Scenario 1: Left Obstacle**
```bash
# Publish left distance < threshold
ros2 topic pub /left_distance std_msgs/msg/Float32 "data: 0.3"
ros2 topic pub /right_distance std_msgs/msg/Float32 "data: 2.0"
```
**Expected**: Left arm raises in RViz, right arm remains neutral.

**Test Scenario 2: Both Obstacles**
```bash
ros2 topic pub /left_distance std_msgs/msg/Float32 "data: 0.2"
ros2 topic pub /right_distance std_msgs/msg/Float32 "data: 0.4"
```
**Expected**: Both arms raise in RViz.

**✅ Checkpoint 4.2**: Verify arm movements match obstacle detections in RViz.

---

## Part 5: Debugging and Optimization (30 minutes)

### Common Issues

**Issue**: Perception agent not receiving sensor data
- **Debug**: `ros2 topic list` → Verify `/left_distance`, `/right_distance` exist
- **Debug**: `ros2 topic info /left_distance` → Check publisher count
- **Fix**: Ensure sensor topics are being published

**Issue**: Control agent commands not moving robot in RViz
- **Debug**: `ros2 topic echo /joint_commands` → Verify commands published
- **Debug**: `ros2 topic echo /joint_states` → Check if joint_state_publisher is processing commands
- **Fix**: Verify joint names in `/joint_commands` match URDF joint names exactly

---

## Lab Deliverables

### Submission Requirements

1. **URDF File** (`lab_humanoid.urdf`): Complete 6-DOF humanoid model
2. **Perception Agent** (`perception_agent.py`): Functional obstacle detection
3. **Control Agent** (`control_agent.py`): Functional arm control
4. **System Demo Video** (2-3 min): Screen recording showing:
   - RViz visualization of robot
   - Publishing sensor data via command line
   - Robot arms responding correctly to obstacles
5. **Lab Report** (1-2 pages): Describe system architecture, challenges encountered, solutions

### Grading Rubric (100 points)

| Component | Points | Criteria |
|-----------|--------|----------|
| URDF Model | 20 | Complete, valid, visualizes correctly |
| Perception Agent | 25 | Correctly detects obstacles, publishes to topic |
| Control Agent | 25 | Commands arms based on detections |
| System Integration | 20 | All components work together |
| Demo Video | 10 | Shows complete system functionality |

---

## Extensions (Optional Challenges)

### Challenge 1: Smooth Arm Motion (Intermediate)
Modify control agent to use velocity commands instead of position commands for smooth motion.

### Challenge 2: Add Head Tracking (Intermediate)
Command `neck` joint to look at direction of detected obstacle.

### Challenge 3: Multi-Level Response (Advanced)
Implement three arm positions based on distance:
- Distance < 0.3m: Fully raised (shoulder -90°)
- 0.3m ≤ Distance < 0.7m: Half raised (shoulder -45°)
- Distance ≥ 0.7m: Neutral (shoulder 0°)

### Challenge 4: State Machine Control (Advanced)
Implement state machine with states: IDLE, DETECTING, AVOIDING, RECOVERING.

---

## Summary

In this lab, you integrated:
1. **URDF modeling** (Chapter 3) to define robot structure
2. **ROS 2 communication** (Chapter 1) via topics for sensor data and joint commands
3. **Python AI agents** (Chapter 2) for perception and control logic
4. **System integration** demonstrating complete autonomous robot behavior

You now have practical experience building ROS 2 robot systems from specification to implementation. Module 2 expands this foundation with advanced simulation environments and multi-sensor fusion.

---

**Congratulations on completing Module 1!** You're ready for [Module 2: overview](../module-2-digital-twin/index.md)

---

**Lab Credits**: Based on ROS 2 Humble tutorials and humanoid robotics best practices
**Last Updated**: 2025-12-18
