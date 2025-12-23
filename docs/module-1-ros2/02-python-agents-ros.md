---
sidebar_position: 3
title: "Chapter 2: Python Agents & ROS Integration"
description: "Integrating Python AI agents with ROS 2 controllers using rclpy for intelligent robot behavior"
---

# Chapter 2: Python Agents & ROS Integration

**Module**: The Robotic Nervous System (ROS 2)
**Estimated Reading Time**: 20 minutes
**Prerequisites**: Chapter 1 (ROS 2 Fundamentals), Python classes and object-oriented programming

---

## Learning Objectives

By the end of this chapter, you will be able to:
- Design Python AI agent classes that integrate seamlessly with ROS 2 using rclpy
- Implement sensor-to-actuator control loops for reactive robot behaviors
- Debug communication issues between Python agents and ROS 2 systems using systematic approaches

---

## Prerequisites

Before starting this chapter, you should:
- Have completed Chapter 1 (understand nodes, topics, publishers, subscribers)
- Know Python object-oriented programming (classes, inheritance, methods)
- Have ROS 2 Humble and Python 3.10+ installed and configured

---

## Introduction

In Chapter 1, you learned how ROS 2 manages communication between robot components using nodes and topics. But how do you connect intelligent decision-making—like object recognition, path planning, or natural language understanding—to physical robot control? This is where **Python AI agents** become essential.

A Python AI agent is a software component that perceives its environment, makes decisions, and takes actions to achieve goals. In robotics, agents bridge high-level reasoning (often implemented in Python using libraries like TensorFlow, PyTorch, or scikit-learn) with low-level control (motors, actuators, ROS 2 controllers). The **rclpy** library enables this integration by providing Python bindings to ROS 2's middleware.

This chapter teaches you to build Python agents that:
1. Subscribe to sensor topics to perceive the robot's state
2. Process data using Python logic (from simple rules to complex AI models)
3. Publish commands to actuator topics to control robot behavior

By the end, you'll create a complete sensor-actuator agent that demonstrates reactive control—the foundation for autonomous humanoid systems in later modules.

---

## Section 1: Python AI Agents in Robotics

### What is a Python AI Agent?

In robotics, an **agent** is any entity that:
- **Perceives** its environment through sensors
- **Reasons** about what actions to take based on perceptions and goals
- **Acts** on the environment through actuators

Python AI agents implement this perceive-reason-act loop using:
- **Python libraries** for AI/ML (NumPy, scikit-learn, TensorFlow, PyTorch)
- **rclpy** to interface with ROS 2 for sensor input and actuator output
- **Control logic** ranging from simple state machines to deep reinforcement learning policies

**Example Use Cases**:
- **Perception Agent**: Subscribes to camera images, runs object detection (YOLO, R-CNN), publishes detected object positions
- **Navigation Agent**: Subscribes to LiDAR scans, plans obstacle-free paths, publishes velocity commands
- **Manipulation Agent**: Subscribes to joint states, computes inverse kinematics, publishes joint position goals

### Why Python for AI in Robotics?

Python dominates AI/ML development because:
1. **Rich Ecosystem**: TensorFlow, PyTorch, OpenCV, scikit-learn, Hugging Face Transformers
2. **Rapid Prototyping**: Fast iteration for testing algorithms
3. **Community Support**: Extensive documentation, tutorials, pre-trained models

However, Python has limitations:
- **Performance**: Slower than C++ for real-time control loops (typically \<1ms required)
- **Global Interpreter Lock (GIL)**: Limits true parallelism in multi-threaded applications

**Best Practice**: Use Python for high-level reasoning (perception, planning) and C++ for time-critical control loops (motor PID controllers, sensor fusion). This chapter focuses on Python agents; Module 3 introduces hybrid Python/C++ systems.

---

## Section 2: The rclpy Library

### What is rclpy?

**rclpy** (ROS Client Library for Python) is the official Python API for ROS 2. It provides Python classes and functions to:
- Create nodes (`rclpy.node.Node`)
- Publish and subscribe to topics (`create_publisher()`, `create_subscription()`)
- Call and provide services (`create_client()`, `create_service()`)
- Manage parameters (`declare_parameter()`, `get_parameter()`)

**rclpy Architecture**:
```
Python Application Code
        ↓
    rclpy (Python)
        ↓
    rcl (C library)
        ↓
    DDS (Data Distribution Service)
```

Your Python code calls rclpy functions, which internally use a C library (rcl) that interfaces with DDS. This layered architecture provides Python's ease of use with DDS's performance and reliability.

### Basic rclpy Pattern

Every rclpy node follows this structure:

```python
import rclpy
from rclpy.node import Node

class MyAgentNode(Node):
    def __init__(self):
        super().__init__('my_agent_node')
        # Initialize publishers, subscribers, timers

def main(args=None):
    rclpy.init(args=args)          # Initialize ROS 2 communication
    node = MyAgentNode()           # Create node instance
    rclpy.spin(node)               # Keep node alive, process callbacks
    node.destroy_node()            # Cleanup
    rclpy.shutdown()               # Shutdown ROS 2

if __name__ == '__main__':
    main()
```

**Key Functions**:
- `rclpy.init()`: Initializes ROS 2 context (must be called once at program start)
- `rclpy.spin(node)`: Blocks and processes callbacks (subscriber callbacks, timer callbacks)
- `node.destroy_node()`: Releases resources
- `rclpy.shutdown()`: Shuts down ROS 2 context

---

## Section 3: Building a Sensor-Actuator Agent

### The Perceive-Reason-Act Loop

Let's build a practical agent that demonstrates sensor-to-actuator control. This agent will:
1. **Perceive**: Subscribe to a sensor topic publishing distance measurements
2. **Reason**: Decide whether to move forward or stop based on distance
3. **Act**: Publish velocity commands to a motor controller topic

**Scenario**: A mobile robot must avoid obstacles. If distance to nearest obstacle > 1.0 meter, move forward at 0.5 m/s. Otherwise, stop.

### Code Example 2.1: Obstacle Avoidance Agent

```python
#!/usr/bin/env python3
"""
Obstacle Avoidance Agent
Purpose: Subscribe to distance sensor, publish velocity commands based on distance
Prerequisites: ROS 2 Humble, Python 3.10+
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32       # Distance sensor message type
from geometry_msgs.msg import Twist    # Velocity command message type


class ObstacleAvoidanceAgent(Node):
    """
    Reactive agent that stops when obstacles are detected within 1 meter.

    Subscribes to:
        /distance_sensor (std_msgs/Float32): Distance to nearest obstacle in meters

    Publishes to:
        /cmd_vel (geometry_msgs/Twist): Velocity commands (linear.x, angular.z)
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_agent')

        # PERCEIVE: Subscribe to distance sensor
        self.subscription = self.create_subscription(
            Float32,
            '/distance_sensor',
            self.distance_callback,
            10
        )

        # ACT: Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Agent state
        self.current_distance = None
        self.safe_distance = 1.0  # meters

        self.get_logger().info('Obstacle Avoidance Agent started')

    def distance_callback(self, msg):
        """
        Called automatically when distance sensor data arrives.

        Implements the perceive-reason-act loop:
        1. PERCEIVE: Read distance from sensor
        2. REASON: Decide action based on distance
        3. ACT: Publish velocity command
        """
        # PERCEIVE: Extract distance from message
        self.current_distance = msg.data

        # REASON: Apply decision logic
        velocity_cmd = self.decide_action(self.current_distance)

        # ACT: Publish velocity command
        self.velocity_publisher.publish(velocity_cmd)

        # Log decision for debugging
        self.get_logger().info(
            f'Distance: {self.current_distance:.2f}m → '
            f'Velocity: {velocity_cmd.linear.x:.2f}m/s'
        )

    def decide_action(self, distance):
        """
        Decision logic: Move forward if safe, stop if obstacle detected.

        Args:
            distance (float): Distance to nearest obstacle in meters

        Returns:
            geometry_msgs/Twist: Velocity command
        """
        velocity = Twist()

        if distance is None:
            # No sensor data yet, remain stopped
            velocity.linear.x = 0.0
        elif distance > self.safe_distance:
            # Safe to move forward
            velocity.linear.x = 0.5  # m/s
        else:
            # Obstacle detected, stop
            velocity.linear.x = 0.0

        velocity.angular.z = 0.0  # No rotation for this simple example
        return velocity


def main(args=None):
    rclpy.init(args=args)
    agent = ObstacleAvoidanceAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts**:
- **Callback-Driven**: `distance_callback()` executes automatically when sensor data arrives
- **Stateless Decision Logic**: `decide_action()` is a pure function (output depends only on input)
- **Separation of Concerns**: Perception (callback), reasoning (decide_action), action (publish) are distinct

---

## Section 4: State Machines for Complex Behaviors

### Beyond Reactive Agents

The obstacle avoidance agent is **reactive**: it responds immediately to current sensor readings without memory. But many robot tasks require **state**:
- "Wait for 5 seconds after detecting an object before moving"
- "Search for an object, then approach it, then grasp it"
- "If stuck for 10 seconds, switch to recovery behavior"

**State machines** enable this by defining:
1. **States**: Distinct behaviors (e.g., IDLE, MOVING, STOPPED)
2. **Transitions**: Conditions that trigger state changes (e.g., distance < 1.0m → STOPPED)
3. **Actions**: What the agent does in each state

### Code Example 2.2: State Machine Agent

```python
#!/usr/bin/env python3
"""
State Machine Navigation Agent
Purpose: Demonstrate stateful agent with IDLE, MOVING, STOPPED states
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from enum import Enum


class RobotState(Enum):
    """States for robot navigation"""
    IDLE = 0
    MOVING = 1
    STOPPED = 2


class StateMachineAgent(Node):
    """
    Agent with state machine for navigation control.

    States:
        IDLE: Waiting for start command
        MOVING: Moving forward while distance > safe_distance
        STOPPED: Stopped due to obstacle, waiting for clearance

    Transitions:
        IDLE → MOVING: distance > safe_distance
        MOVING → STOPPED: distance <= safe_distance
        STOPPED → MOVING: distance > safe_distance + 0.5 (hysteresis)
    """

    def __init__(self):
        super().__init__('state_machine_agent')

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Float32, '/distance_sensor', self.distance_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # State machine
        self.state = RobotState.IDLE
        self.safe_distance = 1.0
        self.resume_distance = 1.5  # Hysteresis to prevent oscillation

        self.get_logger().info(f'State Machine Agent started in state: {self.state.name}')

    def distance_callback(self, msg):
        distance = msg.data
        new_state = self.update_state(distance)

        if new_state != self.state:
            self.get_logger().info(f'State transition: {self.state.name} → {new_state.name}')
            self.state = new_state

        velocity = self.execute_state_action()
        self.velocity_publisher.publish(velocity)

    def update_state(self, distance):
        """
        State transition logic.

        Returns:
            RobotState: New state based on current state and distance
        """
        if self.state == RobotState.IDLE:
            if distance > self.safe_distance:
                return RobotState.MOVING

        elif self.state == RobotState.MOVING:
            if distance <= self.safe_distance:
                return RobotState.STOPPED

        elif self.state == RobotState.STOPPED:
            if distance > self.resume_distance:
                return RobotState.MOVING

        return self.state  # No transition

    def execute_state_action(self):
        """
        Execute action for current state.

        Returns:
            geometry_msgs/Twist: Velocity command for current state
        """
        velocity = Twist()

        if self.state == RobotState.IDLE:
            velocity.linear.x = 0.0
        elif self.state == RobotState.MOVING:
            velocity.linear.x = 0.5
        elif self.state == RobotState.STOPPED:
            velocity.linear.x = 0.0

        return velocity


def main(args=None):
    rclpy.init(args=args)
    agent = StateMachineAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Advanced Concepts**:
- **Enum for States**: Type-safe state representation
- **Hysteresis**: `resume_distance > safe_distance` prevents rapid oscillation between states
- **Separation of State Logic**: `update_state()` handles transitions, `execute_state_action()` handles actions

---

## Section 5: Debugging Python-ROS Integration

### Common Issues and Solutions

| Issue | Symptom | Cause | Solution |
|-------|---------|-------|----------|
| **Node not visible** | `ros2 node list` doesn't show node | Node crashed or not started | Check terminal for error messages; verify `rclpy.init()` called |
| **No messages received** | Subscriber callback never executes | Topic name mismatch or no publisher | Use `ros2 topic list` and `ros2 topic info <topic>` to verify |
| **Messages delayed** | Callbacks lag behind real-time | Processing too slow in callback | Move heavy computation to separate thread or timer callback |
| **Import errors** | `ModuleNotFoundError: No module named 'rclpy'` | ROS 2 environment not sourced | Run `source /opt/ros/humble/setup.bash` |

### Debugging Workflow

1. **Verify Node is Running**:
   ```bash
   ros2 node list
   # Should show your node name
   ```

2. **Check Topic Connections**:
   ```bash
   ros2 topic info /distance_sensor
   # Should show: 1 publisher, 1 subscriber
   ```

3. **Monitor Message Flow**:
   ```bash
   ros2 topic echo /distance_sensor
   # Should display incoming messages
   ```

4. **Inspect Message Types**:
   ```bash
   ros2 interface show std_msgs/msg/Float32
   # Shows message structure
   ```

5. **Test with Manual Publishing**:
   ```bash
   ros2 topic pub /distance_sensor std_msgs/msg/Float32 "data: 0.5"
   # Manually publish test message
   ```

---

## Practice Exercises

### Exercise 2.1: Parameterized Safe Distance (Beginner)
**Objective**: Modify `ObstacleAvoidanceAgent` to read `safe_distance` from a ROS 2 parameter instead of hardcoding it.

**Hint**: Use `self.declare_parameter('safe_distance', 1.0)` in `__init__()` and `self.get_parameter('safe_distance').value` to read it.

**Estimated Time**: 15 minutes

---

### Exercise 2.2: Multi-Level Speed Control (Intermediate)
**Objective**: Enhance the agent to use three speed levels:
- Distance > 2.0m: Fast (0.8 m/s)
- 1.0m < Distance ≤ 2.0m: Medium (0.5 m/s)
- Distance ≤ 1.0m: Stop (0.0 m/s)

**Estimated Time**: 20 minutes

---

### Exercise 2.3: Emergency Stop State (Advanced)
**Objective**: Add an EMERGENCY_STOP state to the state machine that:
- Triggers when distance < 0.3m
- Publishes negative velocity (-0.2 m/s) to back up
- Transitions to STOPPED once distance > 0.5m

**Estimated Time**: 30 minutes

---

## Summary

This chapter introduced Python AI agents integrated with ROS 2:

1. **Python AI Agents** implement perceive-reason-act loops, bridging high-level reasoning with robot control
2. **rclpy** provides Python bindings to ROS 2, enabling easy integration of AI/ML libraries with robot middleware
3. **Sensor-Actuator Patterns** demonstrate reactive control using subscriber callbacks and publisher commands
4. **State Machines** enable complex, stateful behaviors beyond simple reactive responses
5. **Systematic Debugging** using ROS 2 CLI tools ensures reliable agent-robot communication

You now have the foundation to build intelligent Python agents that control robot behavior. Chapter 3 introduces URDF modeling to define the physical structure of humanoid robots, which your agents will control.

---

## Further Reading

- **rclpy API Documentation**: https://docs.ros2.org/latest/api/rclpy/ - Complete Python API reference
- **State Machines in ROS 2**: Koubaa, A. (2020). "Robot Operating System (ROS): The Complete Reference (Volume 4)" - Chapter on behavior trees and state machines (peer-reviewed)
- **Python for Robotics**: Relevant for understanding Python performance considerations in real-time systems

---

**Next Chapter**: [Chapter 3: URDF for Humanoids](./03-urdf-humanoids.md) - Learn to model robot physical structure
