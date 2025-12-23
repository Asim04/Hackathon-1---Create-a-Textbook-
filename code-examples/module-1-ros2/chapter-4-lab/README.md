# Chapter 4: Hands-On Lab - Building a ROS 2 Robot

Complete integration lab combining ROS 2 nodes, Python AI agents, and URDF models.

---

## Lab Overview

This lab demonstrates a complete autonomous humanoid robot system with:
- **6-DOF Humanoid URDF Model**: Torso, head, left/right arms (shoulder + elbow joints)
- **Perception Agent**: Processes distance sensors, publishes obstacle detections
- **Control Agent**: Commands arm movements to avoid detected obstacles
- **ROS 2 Integration**: Topics connecting all components

**System Architecture**:
```
Sensor Publishers (simulated)
        ↓
    /left_distance, /right_distance topics
        ↓
Perception Agent (Python)
        ↓
    /obstacle_detected topic
        ↓
Control Agent (Python)
        ↓
    /joint_commands topic
        ↓
Joint State Publisher → Robot State Publisher → RViz
```

---

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Packages**:
  - `robot-state-publisher`
  - `joint-state-publisher` (without GUI for this lab)
  - `rviz2`

### Installation Check

```bash
source /opt/ros/humble/setup.bash

# Install required packages
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-rviz2
```

---

## Files in This Directory

| File | Purpose |
|------|---------|
| `lab_humanoid.urdf` | 6-DOF upper-body humanoid model |
| `perception_agent.py` | Obstacle detection from distance sensors |
| `control_agent.py` | Arm control based on obstacle detection |
| `README.md` | This file - setup and testing instructions |

---

## Quick Start Guide

### Step 1: Verify URDF Model

```bash
# Check URDF syntax
ros2 run urdf_tutorial check_urdf lab_humanoid.urdf
```

**Expected Output**:
```
robot name is: lab_humanoid
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  torso
        child(1):  head
        child(2):  right_upper_arm
            child(1):  right_forearm
                child(1):  right_hand
        child(3):  left_upper_arm
            child(1):  left_forearm
                child(1):  left_hand
```

---

### Step 2: Launch Visualization

Open **4 terminals** and run these commands:

```bash
# Terminal 1: Robot State Publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat lab_humanoid.urdf)"

# Terminal 2: Joint State Publisher
ros2 run joint_state_publisher joint_state_publisher \
  --ros-args -p source_list:="['/joint_commands']"

# Terminal 3: RViz2
rviz2
```

**RViz Configuration**:
1. Add → RobotModel
2. Fixed Frame → `base_link`
3. Verify humanoid appears (gray torso, beige head, blue arms, green forearms, red hands)

---

### Step 3: Start Python Agents

In **2 new terminals**:

```bash
# Terminal 4: Perception Agent
python3 perception_agent.py

# Terminal 5: Control Agent
python3 control_agent.py
```

**Expected Output** (Perception Agent):
```
[INFO] [perception_agent]: Perception Agent started
[INFO] [perception_agent]: Detection threshold: 0.5m
[INFO] [perception_agent]: Waiting for sensor data...
```

**Expected Output** (Control Agent):
```
[INFO] [control_agent]: Control Agent started
[INFO] [control_agent]: Neutral shoulder: 0.0°
[INFO] [control_agent]: Raised shoulder: -45.0°
[INFO] [control_agent]: Elbow angle: 90.0°
```

---

## Testing the System

### Test 1: Left Obstacle Detection

```bash
# Terminal 6: Publish left obstacle (close)
ros2 topic pub --once /left_distance std_msgs/msg/Float32 "data: 0.3"

# Terminal 7: Publish right safe distance (far)
ros2 topic pub --once /right_distance std_msgs/msg/Float32 "data: 2.0"
```

**Expected Behavior**:
- Perception Agent logs: `Obstacle detected: left (Left: 0.30m, Right: 2.00m)`
- Control Agent logs: `Command: left  → L_shoulder=-45.0°, R_shoulder=  0.0°`
- **RViz**: Left arm raises, right arm remains at side

---

### Test 2: Right Obstacle Detection

```bash
ros2 topic pub --once /left_distance std_msgs/msg/Float32 "data: 2.5"
ros2 topic pub --once /right_distance std_msgs/msg/Float32 "data: 0.4"
```

**Expected Behavior**:
- Detection: `right`
- **RViz**: Right arm raises, left arm at side

---

### Test 3: Both Obstacles

```bash
ros2 topic pub --once /left_distance std_msgs/msg/Float32 "data: 0.2"
ros2 topic pub --once /right_distance std_msgs/msg/Float32 "data: 0.3"
```

**Expected Behavior**:
- Detection: `both`
- **RViz**: Both arms raise

---

### Test 4: No Obstacles

```bash
ros2 topic pub --once /left_distance std_msgs/msg/Float32 "data: 3.0"
ros2 topic pub --once /right_distance std_msgs/msg/Float32 "data: 3.0"
```

**Expected Behavior**:
- Detection: `none` (no log output - only obstacles are logged)
- **RViz**: Both arms return to neutral (sides)

---

## Continuous Testing (Simulating Moving Obstacles)

For more realistic testing, publish at regular intervals:

```bash
# Terminal 6: Continuous left distance (1 Hz)
ros2 topic pub --rate 1 /left_distance std_msgs/msg/Float32 "data: 0.3"

# Terminal 7: Continuous right distance (1 Hz)
ros2 topic pub --rate 1 /right_distance std_msgs/msg/Float32 "data: 2.0"
```

Now watch the logs continuously update and arms maintain raised position.

**Change distance values** to simulate obstacle approaching/receding:
```bash
# Obstacle getting closer
ros2 topic pub --rate 1 /left_distance std_msgs/msg/Float32 "data: 0.1"

# Obstacle moving away
ros2 topic pub --rate 1 /left_distance std_msgs/msg/Float32 "data: 1.5"
```

---

## System Verification Checklist

Use this checklist to verify complete system integration:

- [ ] URDF passes `check_urdf` validation
- [ ] Robot appears correctly in RViz (all links visible)
- [ ] Perception agent starts without errors
- [ ] Control agent starts without errors
- [ ] Left obstacle (0.3m) → Left arm raises in RViz
- [ ] Right obstacle (0.3m) → Right arm raises in RViz
- [ ] Both obstacles → Both arms raise in RViz
- [ ] No obstacles (2.0m) → Both arms neutral in RViz
- [ ] Detection logs appear in perception agent terminal
- [ ] Command logs appear in control agent terminal

---

## Debugging Common Issues

### Issue: Robot doesn't appear in RViz

**Causes**:
1. RobotModel display not added
2. Wrong fixed frame
3. robot_state_publisher not running

**Solutions**:
```bash
# Check robot_state_publisher is running
ros2 node list
# Should show: /robot_state_publisher

# Check robot_description parameter
ros2 param get /robot_state_publisher robot_description
# Should show URDF content

# In RViz: Add → RobotModel, Fixed Frame → base_link
```

---

### Issue: Arms don't move when publishing sensor data

**Cause**: Pipeline broken somewhere (perception → detection → control → joints)

**Debug Steps**:

```bash
# 1. Verify perception agent receives sensor data
ros2 topic echo /left_distance
# Should show: data: 0.3 (or your published value)

# 2. Verify perception publishes detections
ros2 topic echo /obstacle_detected
# Should show: data: 'left' (or 'right', 'both', 'none')

# 3. Verify control publishes joint commands
ros2 topic echo /joint_commands
# Should show JointState with position array

# 4. Verify joint_state_publisher processes commands
ros2 topic echo /joint_states
# Should show JointState with updated positions
```

**Solution**: If any step shows no data, restart that node and check for error messages.

---

### Issue: "No module named 'sensor_msgs'"

**Cause**: ROS 2 message packages not installed

**Solution**:
```bash
sudo apt install ros-humble-sensor-msgs
source /opt/ros/humble/setup.bash
```

---

### Issue: Joints names mismatch error

**Symptom**: Control agent publishes, but joints don't update

**Cause**: Joint names in `control_agent.py` don't match URDF

**Solution**: Verify joint names match exactly (case-sensitive):
```bash
# Check URDF joint names
grep -o 'joint name="[^"]*"' lab_humanoid.urdf

# Should match control_agent.py line ~75:
# joint_cmd.name = ['left_shoulder_pitch', 'left_elbow', 'right_shoulder_pitch', 'right_elbow']
```

---

## Understanding the System

### Perception Agent Logic

```python
def process_detection(self):
    left_obstacle = self.left_distance < 0.5  # threshold
    right_obstacle = self.right_distance < 0.5

    if left_obstacle and right_obstacle:
        detection = "both"
    elif left_obstacle:
        detection = "left"
    elif right_obstacle:
        detection = "right"
    else:
        detection = "none"

    self.detection_pub.publish(String(data=detection))
```

**Key Points**:
- Threshold = 0.5m (configurable in `__init__`)
- Runs at 10 Hz (timer in `__init__`)
- Binary detection (obstacle present/absent, not distance)

---

### Control Agent Logic

```python
def detection_callback(self, msg):
    if msg.data == "left":
        left_shoulder = -0.785  # -45° in radians
        right_shoulder = 0.0
    # ... similar for "right", "both", "none"

    joint_cmd = JointState()
    joint_cmd.name = ['left_shoulder_pitch', 'left_elbow', ...]
    joint_cmd.position = [left_shoulder, 1.571, right_shoulder, 1.571]
    self.joint_pub.publish(joint_cmd)
```

**Key Points**:
- Reactive control (immediate response to detection)
- Position commands (not velocity or effort)
- Elbow always bent 90° (1.571 rad) for visual clarity

---

## Extension Challenges

### Challenge 1: Adjustable Threshold (Beginner)

Modify perception agent to read threshold from ROS 2 parameter:

```python
# In PerceptionAgent.__init__():
self.declare_parameter('threshold', 0.5)
self.detection_threshold = self.get_parameter('threshold').value
```

**Test**:
```bash
python3 perception_agent.py --ros-args -p threshold:=1.0
```

---

### Challenge 2: Multi-Level Arm Position (Intermediate)

Implement three arm positions based on distance:
- < 0.3m: Fully raised (shoulder -90°)
- 0.3m-0.7m: Half raised (shoulder -45°)
- ≥ 0.7m: Neutral (shoulder 0°)

**Hint**: Modify `control_agent.py` detection_callback to check distance ranges instead of binary obstacle detection. You'll need to change `/obstacle_detected` message type to Float32 (distance) instead of String.

---

### Challenge 3: Head Tracking (Intermediate)

Command `neck` joint to look toward detected obstacle:
- Left obstacle → neck rotates left (positive Z-axis)
- Right obstacle → neck rotates right (negative Z-axis)

**Hint**: Add `'neck'` to `joint_cmd.name` array and appropriate position to `joint_cmd.position`.

---

### Challenge 4: Smooth Motion (Advanced)

Use velocity control for smooth arm motion instead of instantaneous position jumps.

**Approach**:
1. Track current vs target joint positions
2. Compute velocity to move toward target: `velocity = K * (target - current)`
3. Publish `joint_cmd.velocity` instead of `joint_cmd.position`

---

## Lab Report Requirements

If submitting this lab for assessment, include:

### 1. System Architecture Diagram
- Show all nodes, topics, message types
- Indicate data flow direction

### 2. Code Modifications (if any)
- Document any changes made to provided code
- Explain rationale for changes

### 3. Testing Results
- Screenshots of RViz showing:
  - Left obstacle → left arm raised
  - Right obstacle → right arm raised
  - Both obstacles → both arms raised
- Console output showing perception and control logs

### 4. Challenges Encountered
- Describe any issues faced
- Explain debugging process and solution

### 5. Extension Implementation (optional)
- If completed any challenges, describe implementation
- Show test results

---

## Additional Resources

- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **sensor_msgs/JointState**: https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html
- **URDF Tutorials**: http://wiki.ros.org/urdf/Tutorials

---

**Lab Tested On**: Ubuntu 22.04, ROS 2 Humble, Python 3.10
**Last Updated**: 2025-12-18
**License**: Educational use
**Estimated Completion Time**: 3-4 hours
