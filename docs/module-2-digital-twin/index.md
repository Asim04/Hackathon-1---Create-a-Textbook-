---
sidebar_position: 1
title: "Module 2: The Digital Twin (Gazebo & Unity)"
description: "Learn to build digital twins for humanoid robots using Gazebo physics simulation, Unity visualization, and ROS 2 integration"
---

# Module 2: The Digital Twin (Gazebo & Unity)

**Duration**: 12-15 hours
**Prerequisites**: Module 1 completion (ROS 2 fundamentals, URDF, Python, C++)

---

## Module Overview

A **digital twin** is a virtual representation of a physical system that mirrors its behavior in real-time through bidirectional data exchange (Grieves & Vickers, 2017). In robotics, digital twins enable safe testing, operator training, and predictive maintenance without risking expensive hardware.

This module teaches digital twin development for humanoid robots using three core technologies:

1. **Gazebo Classic 11**: Physics-based simulation engine for realistic robot dynamics, collisions, and sensor modeling
2. **Unity 2021.3 LTS**: Game engine for photo-realistic 3D visualization and human-robot interaction scenarios
3. **ROS 2 Humble**: Middleware connecting Gazebo (physics) and Unity (visualization) through standardized message passing

You will learn to synchronize physics simulation with high-fidelity rendering, simulate sensors (LiDAR, depth cameras, IMUs) with noise models, and validate digital twin accuracy against real-world specifications.

---

## Learning Objectives

By completing this module, you will be able to:

1. **Configure Gazebo physics engines** (ODE, Bullet) with parameters (gravity, timestep, friction) for stable humanoid simulation
2. **Integrate Unity with ROS 2** using ROS-TCP-Connector for real-time visualization of simulated robots
3. **Simulate sensors** (LiDAR, RGB-D cameras, IMU) with Gazebo plugins and Gaussian noise models matching real-world datasheets
4. **Build a complete digital twin** synchronizing Gazebo physics, Unity rendering, sensor streams, and control loops with \<50ms latency
5. **Validate simulation accuracy** by comparing physics outcomes (contact forces, trajectories) against analytical predictions

---

## Module Structure

### Chapter 1: Gazebo Physics Simulation
**Duration**: 3-4 hours
**Word Count**: 1,300 words

Learn how Gazebo's physics engines simulate robot dynamics through numerical integration. Configure gravity, timesteps, and contact properties (friction, restitution) to match real-world materials. Integrate Gazebo with ROS 2 for model spawning and joint control.

**Key Topics**:
- ODE vs Bullet vs DART physics engines
- Physics parameters: `max_step_size`, `real_time_factor`, solver iterations
- Contact properties: friction coefficients (`mu1`, `mu2`), contact stiffness/damping
- ROS 2 integration: `gazebo_ros_pkgs`, `spawn_entity.py`, joint controllers

**Deliverables**:
- Gazebo world file with physics configuration
- Launch file for ROS 2 + Gazebo integration
- Validation: Robot falls under gravity at 9.81 m/s², collides with ground without penetration

---

### Chapter 2: Unity for High-Fidelity Simulation
**Duration**: 3-4 hours
**Word Count**: 1,400 words

Set up Unity 2021.3 LTS as a digital twin visualization platform. Install ROS-TCP-Connector to subscribe to ROS 2 topics (`/joint_states`) and render robot motion in real-time. Configure lighting, materials, and post-processing for photo-realistic output suitable for operator training.

**Key Topics**:
- Unity project setup (2021.3 LTS, Universal Render Pipeline)
- ROS-TCP-Connector installation and ROSConnection configuration
- Articulation Body components for robotic joints (vs standard Rigidbody)
- Synchronizing Unity transforms with Gazebo joint states (\<50ms latency)

**Deliverables**:
- Unity scene with robot GameObject hierarchy
- C# script subscribing to `/joint_states` and updating Articulation Bodies
- Validation: Unity robot mirrors Gazebo joint angles within 50ms, 30+ FPS frame rate

---

### Chapter 3: Simulating Sensors
**Duration**: 3-4 hours
**Word Count**: 1,300 words

Add sensor simulation to the digital twin using Gazebo plugins. Configure LiDAR (ray sensors), depth cameras (RGB-D), and IMUs with update rates and noise models matching real-world datasheets (Velodyne VLP-16, Intel RealSense D435, BNO055). Visualize sensor data in RViz and validate against specifications.

**Key Topics**:
- Gazebo sensor plugins: `libgazebo_ros_ray_sensor.so`, `libgazebo_ros_camera.so`, `libgazebo_ros_imu_sensor.so`
- Gaussian noise models: mean, standard deviation (σ = 0.01m for LiDAR, 0.02m for depth)
- ROS 2 message types: `sensor_msgs/LaserScan`, `sensor_msgs/Image`, `sensor_msgs/Imu`
- Sensor validation: range accuracy, update rate (10 Hz LiDAR, 30 Hz depth, 100 Hz IMU)

**Deliverables**:
- URDF with sensor plugins (LiDAR, depth camera, IMU)
- Launch file spawning robot with sensors in Gazebo
- Validation: Sensors publish to correct topics, data within specified ranges and noise levels

---

### Chapter 4: Lab - Building a Digital Twin
**Duration**: \4-6 hours
**Word Count**: 1,800 words

Integrate Gazebo, Unity, ROS 2, sensors, and control into a complete digital twin system. Launch Gazebo for physics simulation, connect Unity for visualization, stream sensor data to ROS 2, and implement a simple joint position controller. Validate synchronization latency and real-time performance.

**Key Topics**:
- System architecture: Gazebo → ROS 2 → Unity data flow
- ROS-TCP-Endpoint server configuration (IP address, port 10000)
- Bidirectional communication: Gazebo publishes state, Unity publishes user input
- Performance metrics: real-time factor ≥0.5x, synchronization latency \<50ms

**Lab Activities**:
1. Launch Gazebo with humanoid robot and sensors
2. Connect Unity visualization via ROS-TCP-Connector
3. Implement joint position controller publishing to `/joint_commands`
4. Validate: Robot moves in both Gazebo and Unity, sensor data streams to RViz, latency \<50ms

**Deliverables**:
- Integrated launch file for Gazebo + ROS-TCP-Endpoint + RViz
- Unity scene with bidirectional ROS 2 communication
- Lab report with screenshots, latency measurements, troubleshooting notes

**Grading Rubric** (100 points):
- Digital Twin Functionality (30 pts): Gazebo and Unity synchronized, real-time factor ≥0.5x
- Sensor Integration (25 pts): All sensors publishing correctly
- Control Implementation (20 pts): Joint commands move robot, \<50ms latency
- Code Quality (15 pts): Clean, commented, follows ROS 2 conventions
- Documentation (10 pts): Lab report with results and analysis

---

## Prerequisites

### Knowledge Prerequisites
- **Module 1 Completion**: ROS 2 nodes, topics, launch files, URDF modeling
- **Python Proficiency**: Writing ROS 2 nodes, data structures (lists, dictionaries)
- **C# Basics** (for Unity): Classes, methods, Unity MonoBehaviour lifecycle (for Chapter 2)
- **Linux Command Line**: Navigating directories, running scripts, package installation

### Software Prerequisites

**Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2

**Required Software**:
1. **ROS 2 Humble Hawksbill** (installed in Module 1)
   ```bash
   ros2 --version
   # Expected: ros2 doctor version 0.10.4
   ```

2. **Gazebo Classic 11**
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control -y
   gazebo --version
   # Expected: Gazebo multi-robot simulator, version 11.x.x
   ```

3. **Unity 2021.3 LTS**
   - Download Unity Hub: [unity.com/download](https://unity.com/download)
   - Install Unity 2021.3.x LTS via Unity Hub

4. **ROS-TCP-Endpoint** (Python package)
   ```bash
   pip3 install ros-tcp-endpoint
   ```

5. **RViz2** (for sensor visualization)
   ```bash
   sudo apt install ros-humble-rviz2 -y
   ```

**Verification**:
```bash
# Test Gazebo launch
ros2 launch gazebo_ros gazebo.launch.py
# Press Ctrl+C to exit

# Test ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint
# Should see: ROS-TCP Endpoint started on 0.0.0.0:10000
# Press Ctrl+C to exit
```

---

## Chapters

1. [Chapter 1: Gazebo Physics Simulation](./gazebo-physics) - Physics engines, gravity, collisions, ROS 2 integration
2. [Chapter 2: Unity for High-Fidelity Simulation](./unity-visualization) - Unity setup, ROS-TCP-Connector, real-time rendering
3. [Chapter 3: Simulating Sensors](./sensor-simulation) - LiDAR, depth cameras, IMUs, noise models
4. [Chapter 4: Lab - Building a Digital Twin](./lab-digital-twin) - Complete system integration

---

## References

- [Module 2 References](./references) - Peer-reviewed publications, technical documentation, datasheets

---

## Module Success Criteria

By the end of this module, you will have:

- ✅ **Functional Digital Twin**: Gazebo and Unity synchronized with \<50ms latency
- ✅ **Physics Validation**: Simulated dynamics match real-world specifications (gravity, friction, contact forces)
- ✅ **Sensor Realism**: LiDAR, depth, and IMU data indistinguishable from real sensors (noise characteristics)
- ✅ **System Integration**: Gazebo (physics) + Unity (visualization) + ROS 2 (middleware) working seamlessly
- ✅ **Validated Performance**: Real-time factor ≥0.5x, 30+ FPS rendering, all ROS 2 topics publishing at specified rates

---

## Estimated Time Investment

| Activity | Time | Details |
|----------|------|---------|
| **Chapter Reading** | 4 hr | 1 hr per chapter (1,300-1,800 words each) |
| **Code Examples** | 4 hr | Running and modifying Gazebo worlds, Unity scenes, sensor configs |
| **Practice Exercises** | 3 hr | 3 exercises per chapter (beginner to advanced) |
| **Lab (Chapter 4)** | 4-6 hr | Complete digital twin integration and validation |
| **Total** | **15-17 hr** | Distributed across 4 chapters |

---

## Tips for Success

1. **Test Early and Often**: Run code examples as you read each section (don't wait until chapter end)
2. **Use Word Budget Templates**: Follow chapter structure to stay within 1,200-1,500 words when extending examples
3. **Validate Physics**: Always compare simulation outcomes (falling times, contact forces) against analytical predictions
4. **Monitor Performance**: Check Gazebo real-time factor and Unity FPS regularly - low values indicate configuration issues
5. **Document Issues**: Keep notes on errors encountered and solutions found (helps with final lab report)

---

## Further Learning

After completing this module, consider exploring:

- **Gazebo Ignition (Gazebo Sim)**: Next-generation simulator with improved rendering and distributed simulation
- **NVIDIA Isaac Sim**: GPU-accelerated physics simulation with ray-traced rendering
- **ROS 2 Control**: Advanced joint controllers (PID, trajectory following, impedance control)
- **Digital Twin State Estimation**: Kalman filters for fusing simulated sensor data
- **Hardware-in-the-Loop**: Connecting real sensors to simulated robots

---

**Next**: [Chapter 1: Gazebo Physics Simulation](./gazebo-physics)

---

**Module Version**: 1.0.0
**Last Updated**: 2025-12-18
**Prerequisites Check**: Module 1 completed, Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, Unity 2021.3 LTS
