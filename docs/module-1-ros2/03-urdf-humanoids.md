---
sidebar_position: 4
title: "Chapter 3: URDF for Humanoids"
description: "Defining humanoid robot structure using URDF with links, joints, sensors, and actuators"
---

# Chapter 3: URDF for Humanoids

**Module**: The Robotic Nervous System (ROS 2)
**Estimated Reading Time**: 25 minutes
**Prerequisites**: Chapter 1 (ROS 2 Fundamentals), basic XML syntax

---

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the URDF format and its role in defining robot physical structure
- Create URDF models with links, joints, sensors, and actuators for humanoid robots
- Visualize and debug URDF models using RViz2 and command-line tools

---

## Prerequisites

Before starting this chapter, you should:
- Understand ROS 2 nodes and topics (Chapter 1)
- Know basic XML syntax (tags, attributes, nesting)
- Have ROS 2 Humble, RViz2, and joint-state-publisher installed

---

## Introduction

In Chapters 1 and 2, you learned how ROS 2 manages software communication and how Python agents make decisions. But how do you tell ROS 2 what your robot physically looks like? How many joints does it have? Where are the sensors mounted? What are the link dimensions and masses?

This is where **URDF** (Unified Robot Description Format) becomes essential. URDF is an XML-based format that describes:
- **Physical structure**: Links (rigid bodies) and joints (connections between links)
- **Kinematics**: How joints move and their limits
- **Dynamics**: Masses, inertias, friction for physics simulation
- **Sensors and actuators**: Cameras, LiDAR, force sensors

Think of URDF as the "blueprint" of your robot. Once you define a URDF model, ROS 2 tools can:
- Visualize the robot in 3D (RViz2)
- Simulate physics interactions (Gazebo)
- Compute forward/inverse kinematics
- Plan collision-free paths

This chapter teaches you to create URDF models for humanoid robots, from simple two-link arms to complete 6-degree-of-freedom (DOF) upper bodies.

---

## Section 1: URDF Fundamentals

### What is URDF?

**URDF (Unified Robot Description Format)** is an XML specification for representing robot kinematic and dynamic properties. It was originally developed for ROS 1 and adopted by ROS 2 due to its widespread use and tool ecosystem.

**URDF Describes**:
1. **Links**: Rigid bodies (e.g., upper arm, forearm, torso)
2. **Joints**: Connections between links (e.g., revolute for rotation, prismatic for sliding)
3. **Visual Properties**: Geometry and appearance for rendering
4. **Collision Properties**: Simplified geometry for collision detection
5. **Inertial Properties**: Mass, center of mass, inertia tensor for physics simulation

**Example Use Cases**:
- Defining a humanoid robot's kinematic chain from torso to fingertips
- Specifying joint limits (e.g., elbow can bend 0° to 150°)
- Mounting sensors (e.g., camera at head position, LiDAR on chest)

### Basic URDF Structure

Every URDF file has this structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <!-- Visual, collision, inertial properties -->
  </link>

  <link name="link_1">
    <!-- Properties for link_1 -->
  </link>

  <!-- Joints connect links -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

**Key Elements**:
- `<robot>`: Root element containing all links and joints
- `<link>`: Defines a rigid body component
- `<joint>`: Defines connection and motion between two links
- `<origin>`: Position (xyz) and orientation (roll-pitch-yaw) relative to parent
- `<axis>`: Rotation/translation axis for joint motion

---

## Section 2: Defining Links

### Link Components

A URDF `<link>` has three optional components:

1. **Visual**: Appearance for rendering (what you see in RViz)
2. **Collision**: Simplified geometry for collision detection (what physics engine uses)
3. **Inertial**: Mass and inertia properties for dynamics simulation

```xml
<link name="forearm">
  <!-- Visual: Rendered appearance -->
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.03" length="0.3"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0"/>
    </material>
  </visual>

  <!-- Collision: Simplified for physics -->
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.03" length="0.3"/>
    </geometry>
  </collision>

  <!-- Inertial: Mass and inertia -->
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <mass value="0.5"/>
    <inertia ixx="0.0038" ixy="0.0" ixz="0.0"
             iyy="0.0038" iyz="0.0" izz="0.00045"/>
  </inertial>
</link>
```

**Geometry Types**:
- `<box>`: Rectangular prism (`size="length width height"`)
- `<cylinder>`: Cylindrical shape (`radius="r" length="l"`)
- `<sphere>`: Spherical shape (`radius="r"`)
- `<mesh>`: 3D model file (`.dae`, `.stl`)

**Material Colors**: RGBA format (Red, Green, Blue, Alpha), values 0.0 to 1.0

---

## Section 3: Defining Joints

### Joint Types

URDF supports six joint types:

| Joint Type | Description | Use Case |
|------------|-------------|----------|
| **revolute** | Rotates around axis with limits | Elbow, shoulder, knee |
| **continuous** | Rotates around axis without limits | Wheels, propellers |
| **prismatic** | Slides along axis | Linear actuators, grippers |
| **fixed** | No movement (rigid connection) | Mounting sensors, connecting body parts |
| **floating** | 6 DOF (3 translation + 3 rotation) | Free-floating objects in simulation |
| **planar** | Moves in plane (2 translation + 1 rotation) | Mobile bases on flat ground |

### Revolute Joint Example

```xml
<joint name="shoulder_pitch" type="revolute">
  <!-- Parent link (fixed reference) -->
  <parent link="torso"/>
  <!-- Child link (moves relative to parent) -->
  <child link="upper_arm"/>

  <!-- Position and orientation of child relative to parent -->
  <origin xyz="0.0 0.2 0.4" rpy="0 0 0"/>

  <!-- Rotation axis in parent frame (1 0 0 = X-axis) -->
  <axis xyz="1 0 0"/>

  <!-- Joint limits -->
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>

  <!-- Dynamics (optional) -->
  <dynamics damping="0.7" friction="0.1"/>
</joint>
```

**Joint Limit Parameters**:
- `lower`/`upper`: Joint position limits (radians for revolute, meters for prismatic)
- `effort`: Maximum force/torque the joint can exert (Nm for revolute, N for prismatic)
- `velocity`: Maximum joint speed (rad/s for revolute, m/s for prismatic)

---

## Section 4: Building a Simple Humanoid Arm

### Design Overview

Let's create a 3-DOF (degree-of-freedom) robotic arm representing a simplified humanoid arm:
1. **Shoulder Pitch**: Raises/lowers arm (forward/backward)
2. **Shoulder Roll**: Moves arm side-to-side
3. **Elbow**: Bends forearm

**Kinematic Chain**:
```
base_link (fixed to world)
    ↓ fixed joint
torso (static body)
    ↓ shoulder_pitch (revolute)
upper_arm
    ↓ shoulder_roll (revolute)
upper_arm_rotated
    ↓ elbow (revolute)
forearm
```

### Code Example 3.1: 3-DOF Humanoid Arm URDF

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_arm">

  <!-- Base Link (World Reference) -->
  <link name="base_link"/>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.13" ixy="0.0" ixz="0.0"
               iyy="0.14" iyz="0.0" izz="0.03"/>
    </inertial>
  </link>

  <!-- Fixed Joint: Base to Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Upper Arm -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.0076" ixy="0.0" ixz="0.0"
               iyy="0.0076" iyz="0.0" izz="0.0016"/>
    </inertial>
  </link>

  <!-- Shoulder Pitch Joint -->
  <joint name="shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <origin xyz="0.0 0.15 0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
    <dynamics damping="0.7" friction="0.1"/>
  </joint>

  <!-- Forearm -->
  <link name="forearm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 1.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.0027" ixy="0.0" ixz="0.0"
               iyy="0.0027" iyz="0.0" izz="0.00045"/>
    </inertial>
  </link>

  <!-- Elbow Joint -->
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0.0" upper="2.356" effort="30" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.05"/>
  </joint>

</robot>
```

**Key Design Decisions**:
- **Origins**: Link visual/collision origins placed at center of geometry
- **Axes**: All joints rotate around X-axis (1 0 0) for pitch motion
- **Limits**: Shoulder ±90° (-1.57 to 1.57 rad), Elbow 0° to 135° (0 to 2.356 rad)
- **Materials**: Color-coded for visual distinction (gray torso, blue upper arm, green forearm)

---

## Section 5: Visualizing URDF in RViz2

### Launching RViz with URDF

To visualize the URDF model:

1. **Save URDF file**: Save the above XML as `simple_humanoid_arm.urdf`

2. **Launch RViz with robot model**:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_humanoid_arm.urdf)"
```

3. **In another terminal, launch RViz**:
```bash
rviz2
```

4. **Configure RViz**:
   - Add → RobotModel
   - Set Fixed Frame to `base_link`
   - You should see the gray torso with blue upper arm and green forearm

### Using Joint State Publisher GUI

To interactively move joints:

```bash
# Terminal 1: Robot State Publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_humanoid_arm.urdf)"

# Terminal 2: Joint State Publisher with GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: RViz2
rviz2
```

The GUI provides sliders to adjust each joint angle in real-time. Move the sliders and watch the arm move in RViz.

---

## Section 6: Adding Sensors to URDF

### Sensor Definition with Gazebo Tags

While pure URDF defines structure, Gazebo simulation requires additional tags for sensors. Here's how to add a camera to the forearm:

```xml
<!-- Inside <robot> tag, after forearm link -->

<!-- Camera Link -->
<link name="camera_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
    <material name="red">
      <color rgba="1.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" ixy="0.0" ixz="0.0"
             iyy="0.00001" iyz="0.0" izz="0.00001"/>
  </inertial>
</link>

<!-- Camera Joint (Fixed to Forearm) -->
<joint name="camera_joint" type="fixed">
  <parent link="forearm"/>
  <child link="camera_link"/>
  <origin xyz="0 0 -0.25" rpy="0 1.57 0"/>
</joint>

<!-- Gazebo Camera Plugin -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <camera_name>arm_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Note**: Gazebo-specific tags (like `<gazebo>` and `<sensor>`) are outside pure URDF spec. Module 2 covers simulation in depth.

---

## Practice Exercises

### Exercise 3.1: Add Shoulder Roll Joint (Intermediate)
**Objective**: Modify the 3-DOF arm to add a shoulder roll joint (side-to-side motion) between torso and upper_arm.

**Hint**: Create intermediate link `upper_arm_base`, add `shoulder_roll` joint rotating around Z-axis (0 0 1).

**Estimated Time**: 25 minutes

---

### Exercise 3.2: Create Mirrored Left Arm (Intermediate)
**Objective**: Duplicate the arm structure to create a left arm at `xyz="0.0 -0.15 0.4"` (negative Y).

**Hint**: Copy link/joint definitions, rename (e.g., `left_upper_arm`, `left_elbow`), adjust Y-position origins.

**Estimated Time**: 30 minutes

---

### Exercise 3.3: Add Gripper End Effector (Advanced)
**Objective**: Add a 2-finger gripper at forearm end with two prismatic joints (slide together/apart).

**Requirements**:
- Two finger links (thin boxes)
- Prismatic joints with limits 0.0 to 0.05m
- Fingers should oppose each other (one at +Y, one at -Y)

**Estimated Time**: 40 minutes

---

## Summary

This chapter introduced URDF for defining robot physical structure:

1. **URDF Format** describes links (rigid bodies), joints (connections), and properties (visual, collision, inertial)
2. **Links** define robot components with geometry, appearance, and mass properties
3. **Joints** connect links and specify motion type (revolute, prismatic, fixed, etc.) with limits
4. **Visualization** using RViz2 and joint_state_publisher_gui enables interactive model inspection
5. **Sensors** can be added to URDF models for simulation (expanded in Module 2)

You now have the skills to define humanoid robot kinematics. Chapter 4 integrates URDF models with ROS 2 controllers and Python agents in a complete hands-on lab.

---

## Further Reading

- **URDF XML Specification**: http://wiki.ros.org/urdf/XML - Official URDF format reference
- **URDF Tutorials**: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html - ROS 2 official URDF tutorials
- **Inertia Calculations**: Laible et al. (2021) "Automated URDF Generation for Robotic Systems" - Methods for computing accurate inertial properties (peer-reviewed)

---

**Next Chapter**: [Chapter 4: Hands-On Lab - Building a ROS 2 Robot](./04-lab-building-ros2-robot.md) - Integrate everything in a complete system
