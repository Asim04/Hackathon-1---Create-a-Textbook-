# Chapter 3: URDF for Humanoids - Code Examples

This directory contains URDF models and visualization scripts for humanoid robot modeling.

---

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2 Version**: Humble Hawksbill
- **Required Packages**:
  - `robot-state-publisher`
  - `joint-state-publisher-gui`
  - `rviz2`

### Installation Check

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install required packages if missing
sudo apt update
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui ros-humble-rviz2

# Verify installation
ros2 pkg list | grep robot_state_publisher
ros2 pkg list | grep joint_state_publisher
```

---

## Files in This Directory

### 1. simple_humanoid_arm.urdf
**Purpose**: Complete 3-DOF humanoid arm model with torso, upper arm, forearm, and hand

**Structure**:
- `base_link`: World reference frame
- `torso`: Gray box (0.3×0.2×0.5m, 5kg mass)
- `upper_arm`: Blue cylinder (r=0.04m, l=0.3m, 1kg mass)
- `forearm`: Green cylinder (r=0.03m, l=0.25m, 0.5kg mass)
- `hand`: Red sphere (r=0.04m, 0.1kg mass)

**Joints**:
- `base_to_torso`: Fixed joint (torso attached to world)
- `shoulder_pitch`: Revolute joint (±90°, X-axis rotation)
- `elbow`: Revolute joint (0° to 135°, X-axis rotation)
- `hand_joint`: Fixed joint (hand attached to forearm end)

**Joint Limits**:
| Joint | Type | Axis | Lower | Upper | Max Effort | Max Velocity |
|-------|------|------|-------|-------|------------|--------------|
| shoulder_pitch | revolute | X (1 0 0) | -1.57 rad (-90°) | 1.57 rad (90°) | 50 Nm | 2.0 rad/s |
| elbow | revolute | X (1 0 0) | 0.0 rad (0°) | 2.356 rad (135°) | 30 Nm | 2.0 rad/s |

### 2. launch_urdf_rviz.sh
**Purpose**: Bash script to launch robot_state_publisher, joint_state_publisher_gui, and RViz2 automatically

**Usage**: See "Quick Start" section below

---

## Quick Start

### Option 1: Automated Launch Script (Recommended)

```bash
# Navigate to this directory
cd code-examples/module-1-ros2/chapter-3/

# Make script executable
chmod +x launch_urdf_rviz.sh

# Run the script
./launch_urdf_rviz.sh
```

**What Happens**:
1. `robot_state_publisher` starts and loads the URDF model
2. `joint_state_publisher_gui` opens with sliders for each joint
3. RViz2 opens (you need to configure it - see "RViz Configuration" below)

### Option 2: Manual Launch (Step-by-Step)

```bash
# Terminal 1: Robot State Publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_humanoid_arm.urdf)"

# Terminal 2: Joint State Publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: RViz2
rviz2
```

---

## RViz Configuration

After launching RViz2:

1. **Add RobotModel Display**:
   - Click "Add" button (bottom left)
   - Select "By display type" tab
   - Choose "RobotModel"
   - Click "OK"

2. **Set Fixed Frame**:
   - In "Global Options" panel
   - Set "Fixed Frame" to `base_link`

3. **Verify Robot Appears**:
   - You should see gray torso, blue upper arm, green forearm, red hand
   - If not visible, check "RobotModel" display is enabled (checkbox)

4. **Adjust View**:
   - Mouse controls:
     - Left-click + drag: Rotate view
     - Middle-click + drag: Pan view
     - Scroll wheel: Zoom in/out

5. **Move Joints**:
   - Use Joint State Publisher GUI sliders
   - Watch robot arm move in RViz in real-time

### Save RViz Configuration (Optional)

To avoid reconfiguring RViz every time:

1. After setup, go to File → Save Config As
2. Save as `simple_arm.rviz` in this directory
3. Next time, launch with: `rviz2 -d simple_arm.rviz`

---

## Testing the URDF Model

### 1. Verify URDF Syntax

```bash
# Check for XML errors
ros2 run urdf_tutorial check_urdf simple_humanoid_arm.urdf
```

**Expected Output**:
```
robot name is: simple_humanoid_arm
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  torso
        child(1):  upper_arm
            child(1):  forearm
                child(1):  hand
```

### 2. Visualize Kinematic Tree

```bash
# Generate visual tree diagram
ros2 run urdf_tutorial urdf_to_graphiz simple_humanoid_arm.urdf
```

Generates `simple_humanoid_arm.pdf` showing the kinematic tree structure.

### 3. Test Joint Limits

In Joint State Publisher GUI:
1. Move `shoulder_pitch` slider to -1.57 (minimum)
2. Upper arm should point backward
3. Move to +1.57 (maximum)
4. Upper arm should point forward
5. Slider should not go beyond these limits

### 4. Check TF Transforms

```bash
# Terminal 1: Keep robot_state_publisher and joint_state_publisher running

# Terminal 2: Echo transform tree
ros2 run tf2_ros tf2_echo base_link hand
```

**Expected Output**: Shows transform (translation + rotation) from `base_link` to `hand`, updated in real-time as you move joints.

---

## Understanding the URDF Structure

### Visual vs Collision vs Inertial

```xml
<link name="forearm">
  <!-- VISUAL: What you see in RViz (detailed geometry, colors) -->
  <visual>
    <geometry><cylinder radius="0.03" length="0.25"/></geometry>
    <material name="green"><color rgba="0.0 1.0 0.0 1.0"/></material>
  </visual>

  <!-- COLLISION: Simplified geometry for physics (often same as visual) -->
  <collision>
    <geometry><cylinder radius="0.03" length="0.25"/></geometry>
  </collision>

  <!-- INERTIAL: Mass and inertia tensor for dynamics simulation -->
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.0027" .../>
  </inertial>
</link>
```

**Why Three Definitions?**
- **Visual**: High-detail meshes for realistic rendering (performance impact)
- **Collision**: Simplified shapes for fast collision detection (cylinders cheaper than complex meshes)
- **Inertial**: Physics simulation needs mass/inertia (Gazebo uses this, RViz ignores it)

### Joint Types Quick Reference

| Type | Motion | Example Use |
|------|--------|-------------|
| `fixed` | None (rigid connection) | Mounting sensors, connecting body segments |
| `revolute` | Rotation with limits | Human joints (elbow, shoulder) |
| `continuous` | Unlimited rotation | Wheels, propellers |
| `prismatic` | Linear sliding | Grippers, linear actuators |

---

## Troubleshooting

### Issue: RViz shows no robot

**Cause**: RobotModel display not added or wrong fixed frame

**Solution**:
1. Verify "RobotModel" display is in left panel (add if missing)
2. Set Fixed Frame to `base_link` in Global Options
3. Check robot_state_publisher is running: `ros2 node list` should show `/robot_state_publisher`

---

### Issue: "No module named 'PyQt5'"

**Cause**: Joint State Publisher GUI requires PyQt5

**Solution**:
```bash
sudo apt install python3-pyqt5
```

---

### Issue: Joints won't move in RViz

**Cause**: Joint State Publisher not running or not publishing

**Solution**:
1. Verify joint_state_publisher_gui is running: `ros2 node list`
2. Check joint states are being published:
```bash
ros2 topic echo /joint_states
```
Should show `position:` array updating as you move sliders.

---

### Issue: URDF check fails with "child link [X] of joint [Y] not found"

**Cause**: Joint references non-existent link name (typo)

**Solution**: Verify `<parent link="...">` and `<child link="...">` exactly match `<link name="...">` tags (case-sensitive).

---

## Practice Exercises Solutions

### Exercise 3.1: Add Shoulder Roll Joint

Create intermediate link between torso and upper_arm:

```xml
<!-- Add after upper_arm link -->
<link name="shoulder_connector">
  <visual>
    <geometry><sphere radius="0.05"/></geometry>
    <material name="yellow"><color rgba="1.0 1.0 0.0 1.0"/></material>
  </visual>
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<!-- Modify shoulder_pitch to connect torso → shoulder_connector -->
<joint name="shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="shoulder_connector"/>
  <!-- ... rest same ... -->
</joint>

<!-- Add new shoulder_roll joint -->
<joint name="shoulder_roll" type="revolute">
  <parent link="shoulder_connector"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Z-axis for side-to-side -->
  <limit lower="-1.57" upper="1.57" effort="40" velocity="2.0"/>
</joint>
```

---

## Extending the Model

### Add a Camera Sensor

```xml
<!-- Camera link at hand position -->
<link name="camera_link">
  <visual>
    <geometry><box size="0.02 0.05 0.02"/></geometry>
    <material name="black"><color rgba="0.0 0.0 0.0 1.0"/></material>
  </visual>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" iyy="0.00001" izz="0.00001" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="hand"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

For Gazebo simulation, add plugin (covered in Module 2).

---

**Code Tested On**: Ubuntu 22.04, ROS 2 Humble
**Last Updated**: 2025-12-18
**License**: Educational use
