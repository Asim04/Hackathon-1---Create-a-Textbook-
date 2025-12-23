---
sidebar_position: 5
title: "Chapter 4: Building a Complete Digital Twin Lab"
description: "Integrate Gazebo physics, Unity visualization, and sensor simulation into a complete digital twin system for autonomous robot navigation."
---

# Chapter 4: Building a Complete Digital Twin Lab

**Module**: The Digital Twin (Gazebo & Unity)
**Estimated Reading Time**: 35 minutes
**Prerequisites**: Chapters 1-3 (Gazebo, Unity, Sensors)

---

## Learning Objectives

By the end of this chapter, you will be able to:
- Design a complete digital twin architecture integrating Gazebo, Unity, and ROS 2
- Build a navigation scenario with physics simulation and photo-realistic visualization
- Validate digital twin synchronization and sensor data flow
- Debug multi-component system issues using systematic troubleshooting

---

## Prerequisites

**Knowledge Prerequisites**:
- Understand Gazebo physics parameters (Chapter 1)
- Configure Unity scenes with ROS-TCP-Connector (Chapter 2)
- Implement sensor plugins with noise models (Chapter 3)
- ROS 2 launch files and multi-node orchestration (Module 1, Chapter 4)

**Software Prerequisites**:
- Gazebo Classic 11: `gazebo --version`
- Unity 2021.3 LTS with ROS-TCP-Connector
- ROS 2 Humble: `ros2 --version`
- Python 3.10+: `python3 --version`

---

## Introduction
## Introduction

A digital twin is a virtual replica of a physical robot that behaves according to real-world
physics, sensor data, and control logic. In humanoid robotics, digital twins are essential
because testing directly on physical robots is expensive, slow, and risky. A well-designed
digital twin allows developers to safely experiment, debug, and improve robot behavior
before deploying it to real hardware.

In this lab chapter, we bring together all major components of the Digital Twin module.
Gazebo is used to simulate realistic physics such as gravity, friction, and collisions.
Unity provides high-fidelity 3D visualization and human-friendly interaction. Simulated
sensors such as LiDAR, depth cameras, and IMUs generate data that closely matches real
hardware. Finally, Python-based ROS 2 nodes close the loop by controlling robot behavior
based on sensor feedback.

By the end of this chapter, students will understand how individual simulation tools are
combined into a single, coherent digital twin system. This system mirrors how modern
robotics teams develop and test autonomous humanoid robots in real-world research and
industry environments.

## 1. Digital Twin System Architecture

A digital twin system in robotics is built using multiple software components that work
together as a single coordinated system. Each component has a clear responsibility, and
all communication between them is handled through ROS 2. This modular design makes the
system easier to understand, test, and extend.

At the core of the architecture is ROS 2, which acts as the nervous system of the robot.
ROS 2 nodes handle communication between the simulator, sensors, control logic, and
visualization tools. Topics are used to continuously stream data such as sensor readings
and robot state, while services and actions are used for structured commands like starting
navigation or resetting the simulation.

Gazebo is responsible for simulating the physical world. It computes gravity, collisions,
friction, and joint dynamics for the humanoid robot. The robot model, defined using URDF,
ensures that the simulated robot has realistic dimensions, mass, and joint limits. Gazebo
publishes sensor data and robot state information directly to ROS 2 topics.

Unity is connected to the system as a visualization and interaction layer. Using the
ROS–TCP–Connector, Unity subscribes to ROS 2 topics and displays the robot and environment
in a high-quality 3D interface. This allows developers and students to visually inspect
robot behavior, sensor alignment, and navigation decisions in real time.

Python-based ROS 2 nodes act as the decision-making layer. These nodes process sensor data,
run navigation or control logic, and send movement commands back to the robot. Together,
Gazebo, Unity, sensors, and ROS 2 form a complete digital twin that closely mirrors a real
humanoid robot system.

## 2. Gazebo–Unity Integration Pipeline

In a complete digital twin lab, Gazebo and Unity serve different but complementary roles.
Gazebo focuses on accurate physics simulation, while Unity focuses on visualization and
human interaction. To build a reliable digital twin, these two tools must be connected in a
structured and synchronized way using ROS 2 as the communication backbone.

The integration pipeline begins in Gazebo, where the robot and environment are simulated.
Gazebo computes physical effects such as gravity, collisions, joint motion, and sensor
behavior. As the simulation runs, Gazebo publishes robot state information, including joint
positions, velocities, and sensor outputs, to ROS 2 topics. These topics represent the
single source of truth for the robot’s current state.

Unity connects to ROS 2 using the ROS–TCP–Connector package. This connector allows Unity to
subscribe to selected ROS 2 topics

## 3. Sensor Simulation and Data Validation

Sensors are a critical part of any digital twin system because autonomous robot behavior
depends entirely on sensor data. In this lab, common humanoid robot sensors such as LiDAR,
depth cameras, and IMUs are simulated inside Gazebo. These simulated sensors are designed
to closely match the behavior of real-world hardware.

Gazebo provides built-in sensor models that can be attached to the robot using URDF and
sensor configuration files. Each sensor publishes data to ROS 2 topics in the same format
used by physical sensors. For example, a LiDAR sensor publishes laser scan messages, while
a depth camera publishes depth images and point clouds. This allows the same control and
navigation code to work in both simulation and real robots.

To make simulation realistic, sensor noise models are enabled. Noise simulates measurement
errors such as small distance inaccuracies, timing delays, and signal fluctuations. These
effects are important because real sensors never produce perfect data. By including noise,
students learn to design algorithms that are robust and reliable under imperfect
conditions.

Data validation is the process of checking whether simulated sensor output matches expected
real-world behavior. This is done by comparing sensor ranges, update rates, and data
patterns against manufacturer specifications or known benchmarks. For example, LiDAR
maximum range and angular resolution can be verified using simple test environments in
Gazebo.

Through sensor simulation and validation, the digital twin becomes a trustworthy testing
environment. Developers gain confidence that behaviors observed in simulation will transfer
to real humanoid robots with minimal adjustment.

## 4. Autonomous Navigation Loop

An autonomous navigation loop connects perception, decision-making, and motion into a
continuous cycle. In the digital twin lab, this loop allows the humanoid robot to sense its
environment, plan actions, and move safely within the simulated world. The same structure
used here is also applied in real-world robotics systems.

The navigation loop begins with sensor input. Simulated LiDAR, depth cameras, and IMU data
are published to ROS 2 topics and continuously updated as the robot moves. This data
provides the robot with information about obstacles, free space, and its own orientation.
Navigation nodes subscribe to these topics to build an internal representation of the
environment.

Based on sensor data, the planning layer determines what action the robot should take
next. Path planning algorithms calculate a safe route from the robot’s current position to
a target goal while avoiding obstacles. In this lab, navigation decisions are executed
inside ROS 2 using standardized interfaces, allowing the same logic to be reused across
different robots and environments.

Once a plan is created, control commands are sent to the robot’s actuators. These commands
control joint movement and base motion in Gazebo, causing the robot to physically move in
the simulated world. As the robot moves, new sensor data is generated, and the navigation
loop repeats continuously.

Unity plays an important role by visualizing this loop in real time. Students can observe
planned paths, robot motion, and obstacle avoidance behavior through a cl

## Lab Summary and Learning Outcomes

In this lab chapter, students built a complete digital twin system by integrating Gazebo,
Unity, ROS 2, and simulated sensors into a single workflow. Each component played a
specialized role: Gazebo provided physics-accurate simulation, Unity delivered clear
visualization, and ROS 2 enabled reliable communication and control.

Students learned how a humanoid robot’s virtual environment can closely mirror real-world
behavior when physics, sensors, and control logic are properly aligned. By validating
sensor data and observing autonomous navigation in simulation, students gained confidence
in testing robot behavior safely before deploying to physical hardware.

After completing this lab, students can design and analyze digital twin systems, understand
the flow of data across simulation tools, and debug autonomous robot behavior using
visual feedback. These skills are essential for modern humanoid robotics development and
form the foundation for more advanced perception and AI-driven control systems.
