---
sidebar_position: 3
title: "Chapter 2: Unity for High-Fidelity Simulation"
description: "Integrate Unity 2021.3 LTS with ROS 2 using ROS-TCP-Connector for photo-realistic robot visualization and human-robot interaction scenarios"
---

# Chapter 2: Unity for High-Fidelity Simulation

**Module**: The Digital Twin (Gazebo & Unity)
**Estimated Reading Time**: 30 minutes
**Prerequisites**: Chapter 1 (Gazebo Physics), Unity basics (GameObjects, Components, Inspector)

---

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure Unity 2021.3 LTS as a digital twin visualization platform for humanoid robots
- Install and configure ROS-TCP-Connector to communicate with ROS 2 topics
- Subscribe to `/joint_states` and update Unity Articulation Bodies in real-time
- Achieve \<50ms synchronization latency between Gazebo physics and Unity rendering

---

## Prerequisites

**Knowledge Prerequisites**:
- Gazebo physics simulation (Chapter 1)
- ROS 2 topics and message types (Module 1, Chapter 1)
- Basic Unity: Scene view, GameObjects, Inspector, Transform components

**Software Prerequisites**:
- Unity 2021.3 LTS installed via Unity Hub
- ROS 2 Humble with running Gazebo simulation (from Chapter 1)
- Verification: Unity Hub shows 2021.3.x installed, Gazebo world running

---

## Introduction

Gazebo provides accurate physics, but its rendering is designed for engineering validation, not demonstrations or human-robot interaction studies. Unity, a game engine used for AAA titles and virtual reality, offers photo-realistic lighting, post-processing effects, and physics-based materials that make robots visually indistinguishable from real hardware.

A digital twin combines both: Gazebo computes physics (forces, collisions, sensor data) while Unity renders the scene for stakeholders, operators, or training scenarios. The ROS-TCP-Connector bridge synchronizes these systems through ROS 2 topics, enabling real-time visualization of simulated robots with \<50ms latency (Unity Technologies, 2021).

In this chapter, you'll set up Unity as a visualization frontend, connect it to Gazebo via ROS 2, and stream joint states to animate a humanoid robot in high-fidelity 3D.

---

## Section 1: Unity for Robotics

### Why Unity for Digital Twins?

Unity advantages over RViz/Gazebo GUI:

1. **Photo-Realistic Rendering**: Physically-based rendering with real-time global illumination, reflections, shadows
2. **Human Assets**: Pre-built models for human-robot interaction
3. **VR/AR Support**: Deploy to Oculus, HoloLens for teleoperation
4. **Cross-Platform**: Windows, Linux, macOS, WebGL

**Trade-off**: Unity's PhysX less accurate than Gazebo ODE/Bullet. Solution: Gazebo for physics, Unity for rendering only.

### Articulation Body vs Rigidbody

Unity provides two physics components for hierarchical objects:

- **Rigidbody**: Independent rigid bodies with collision detection. Use for simple objects (boxes, balls).
- **Articulation Body**: Reduced-coordinate articulated systems with joint constraints. **Required for robots** to maintain kinematic chain integrity.

**Key Difference**: Articulation Bodies use maximal coordinate reduction (Featherstone algorithm) for stable joint simulation, preventing drift in long kinematic chains (Unity Technologies, 2021).

**Example**:
```csharp
// Attach to each robot link
ArticulationBody joint = GetComponent<ArticulationBody>();
joint.jointType = ArticulationJointType.RevoluteJoint; // or PrismaticJoint
joint.anchorRotation = Quaternion.Euler(0, 0, 0); // Joint axis
```

### Rendering Pipeline

Unity 2021.3 LTS supports two rendering pipelines:

1. **Built-in Pipeline**: Legacy, simple lighting, lower performance
2. **Universal Render Pipeline (URP)**: Modern, optimized for real-time, recommended for robotics

**Recommendation**: Use **URP** for digital twins (better shadows, post-processing, 30+ FPS on mid-range hardware).

---

## Section 2: Installing ROS-TCP-Connector

### Package Overview

ROS-TCP-Connector provides:
- TCP socket communication with ROS 2
- JSON message serialization
- Publisher/Subscriber API

**Architecture**: Unity ↔ TCP :10000 ↔ ROS-TCP-Endpoint ↔ ROS 2

### Installation Steps

#### 1. Create Unity Project
```
Unity Hub > New Project
Template: 3D (Core) or 3D (URP)
Project Name: DigitalTwinVisualization
Unity Version: 2021.3.x LTS
```

#### 2. Import ROS-TCP-Connector
```
Unity Editor > Window > Package Manager
+ (Plus) > Add package from git URL
URL: https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
Add
```

**Verification**: Package Manager shows "ROS TCP Connector" under "In Project" (takes ~60 seconds).

#### 3. Configure ROSConnection
```
Robotics > ROS Settings (opens Inspector)
ROS IP Address: 127.0.0.1 (localhost, or remote IP)
ROS Port: 10000 (default)
Protocol: ROS 2
Show HUD: ✓ (displays connection status overlay)
```

**Testing**: Press Play in Unity. HUD should show "Connecting..." then "Connected" (green) when ROS-TCP-Endpoint is running.

---

## Section 3: ROS-Unity Communication

### Message Serialization

ROS-TCP-Connector serializes messages as JSON for cross-platform compatibility. Standard types are pre-defined:
- `sensor_msgs/JointState` → `JointStateMsg.cs`
- `geometry_msgs/Pose` → `PoseMsg.cs`
- `std_msgs/String` → `StringMsg.cs`

**Custom Messages**: Generate C# classes with `msg-gen` tool (see ROS-TCP-Connector documentation).

### Topic Subscription

**Pattern**: Register callback function for ROS 2 topic, triggered on message arrival.

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

void Start()
{
    ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
        "/joint_states",           // ROS 2 topic name
        OnJointStateReceived       // Callback function
    );
}

void OnJointStateReceived(JointStateMsg msg)
{
    // Update robot joints here
    Debug.Log($"Received {msg.name.Length} joint states");
}
```

### Latency Considerations

SC-002 requires \<50ms latency. Factors:

1. **Network**: 5-20ms localhost, 20-100ms remote
2. **Serialization**: ~1-5ms JSON encoding
3. **Frame Rate**: 30 FPS = 33ms, 60 FPS = 16ms

**Optimization**:
- Run ROS-TCP-Endpoint on same machine (minimize delay)
- Target 60 FPS (Edit > Project Settings > Quality > VSync: Every V Blank)
- Use `Application.targetFrameRate = 60`

### QoS Policies

ROS 2 Quality of Service affects reliability vs latency:
- **Best Effort**: Low latency, tolerated packet loss (joint states, high-rate data)
- **Reliable**: Guaranteed delivery, higher latency (commands)

ROS-TCP-Connector defaults to Best Effort for real-time visualization (Juliani et al., 2020).

---

## Section 4: Synchronizing Robot State

### Subscribing to Joint States

ROS 2 `joint_state_publisher` publishes to `/joint_states` (type: `sensor_msgs/JointState`) at ~1 kHz. Unity subscribes and updates Articulation Bodies every frame.

**Workflow**:
1. Gazebo computes physics → publishes `/joint_states`
2. ROS-TCP-Endpoint forwards to Unity via TCP
3. Unity callback receives message
4. Script updates Articulation Body joint targets
5. Unity physics engine interpolates to target (smooth motion)

### Updating Articulation Bodies

**Key Concept**: Articulation Bodies use `xDrive.target` (for revolute/prismatic joints) to set desired joint angle/position. Unity's physics engine applies forces to reach target.

```csharp
void OnJointStateReceived(JointStateMsg msg)
{
    for (int i = 0; i < msg.name.Length; i++)
    {
        string jointName = msg.name[i];
        double position = msg.position[i]; // Radians

        ArticulationBody joint = FindJoint(jointName);
        if (joint != null)
        {
            var drive = joint.xDrive;
            drive.target = (float)(position * Mathf.Rad2Deg); // Convert to degrees
            joint.xDrive = drive;
        }
    }
}
```

### Timestamp Handling

`JointStateMsg.header.stamp` contains ROS 2 time (seconds, nanoseconds). Use for latency measurement:

```csharp
void OnJointStateReceived(JointStateMsg msg)
{
    double rosTime = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    double unityTime = Time.time; // Unity elapsed time
    double latency = unityTime - rosTime; // Approximate latency

    if (latency > 0.05) // 50ms threshold
        Debug.LogWarning($"High latency: {latency * 1000:F1}ms");
}
```

**Note**: Requires synchronized clocks (use NTP or ROS 2 `/clock` topic).

### Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Joints don't move | Joint names mismatch Unity GameObject names | Ensure URDF joint names match Unity hierarchy |
| Jittery motion | xDrive stiffness too high | Reduce `drive.stiffness` to 1000-10000 |
| Slow response | xDrive damping too high | Reduce `drive.damping` to 100-1000 |
| Connection timeout | ROS-TCP-Endpoint not running | Start endpoint: `ros2 run ros_tcp_endpoint default_server_endpoint` |
| High latency (>50ms) | Unity frame rate too low | Increase target frame rate to 60 FPS |

---

## Practice Exercises

### Exercise 2.1: Change Unity Scene Lighting (Beginner)
**Objective**: Modify directional light color and intensity to simulate different environments (indoor, outdoor, sunset).

**Hint**: Select "Directional Light" in Hierarchy, change Color (yellow for sunset) and Intensity (0.5-2.0). Observe how robot appearance changes.

**Estimated Time**: 15 minutes

---

### Exercise 2.2: Add Second Robot with Namespaced Topics (Intermediate)
**Objective**: Spawn two robots in Gazebo with namespaces `/robot1`, `/robot2`, visualize both in Unity.

**Hint**: Create two ROS subscribers (`/robot1/joint_states`, `/robot2/joint_states`), instantiate two robot GameObjects, map subscriptions to respective Articulation Bodies.

**Estimated Time**: 45 minutes

---

### Exercise 2.3: Implement Bidirectional Communication (Advanced)
**Objective**: Publish Unity user input (keyboard arrow keys) to ROS 2 topic `/cmd_vel` to control robot in Gazebo.

**Hint**: Use `ROSConnection.GetOrCreateInstance().Publish<TwistMsg>("/cmd_vel", twistMsg)` in Unity's `Update()` loop when keys pressed. Subscribe in ROS 2 node to move robot.

**Estimated Time**: 60 minutes

---

## Summary

This chapter covered:
1. **Unity for robotics**: Articulation Bodies for kinematic chains, URP for photo-realistic rendering
2. **ROS-TCP-Connector setup**: Package import via git URL, ROSConnection configuration (IP, port, protocol)
3. **Message communication**: JSON serialization, topic subscription with callbacks, latency optimization (\<50ms target)
4. **Joint state synchronization**: Subscribe to `/joint_states`, update Articulation Body `xDrive.target`, timestamp-based latency monitoring
5. **Troubleshooting**: Joint name matching, drive parameter tuning, connection diagnostics

**Forward Reference**: In Chapter 3, you'll add sensor visualization (LiDAR point clouds, depth camera streams) to the Unity scene, completing the perception layer of the digital twin.

---

## Further Reading

- **Unity Technologies. (2021)**. *Unity 2021.3 LTS documentation - Articulation Body*. Retrieved from https://docs.unity3d.com/2021.3/Documentation/Manual/class-ArticulationBody.html - Official guide to reduced-coordinate articulated systems for robotics
- **Unity Robotics Hub**. Retrieved from https://github.com/Unity-Technologies/Unity-Robotics-Hub - ROS-TCP-Connector tutorials, URDF importer, pick-and-place demos
- **Juliani, A., et al. (2020)**. Unity: A general platform for intelligent agents. *arXiv preprint arXiv:1809.02627*. - Unity ML-Agents for robot learning and sim-to-real transfer

---

**Next Chapter**: [Chapter 3: Simulating Sensors](./sensor-simulation)
