# Chapter 2: Python Agents & ROS Integration - Code Examples

This directory contains Python AI agent examples demonstrating integration with ROS 2 using rclpy.

---

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2 Version**: Humble Hawksbill
- **Python**: 3.10 or later

### Installation Check

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify rclpy is available
python3 -c "import rclpy; print('rclpy OK')"
```

---

## Code Examples

### 1. obstacle_avoidance_agent.py - Reactive Agent

**Purpose**: Demonstrates perceive-reason-act loop for obstacle avoidance using sensor-to-actuator pattern.

**Concepts Covered**:
- Subscribing to sensor topics (Float32 distance data)
- Publishing actuator commands (Twist velocity messages)
- Reactive control (immediate response to sensor input)
- Pure function design for decision logic

**How to Run**:

Since this agent requires a distance sensor and velocity controller, we'll simulate them:

```bash
# Terminal 1: Run the agent
python3 obstacle_avoidance_agent.py

# Terminal 2: Simulate distance sensor (safe distance - robot moves)
ros2 topic pub --rate 1 /distance_sensor std_msgs/msg/Float32 "data: 2.0"

# Terminal 3: Monitor velocity commands
ros2 topic echo /cmd_vel
```

**Expected Behavior**:
- When distance > 1.0m: Publishes velocity 0.5 m/s (moving forward)
- When distance ≤ 1.0m: Publishes velocity 0.0 m/s (stopped)

**Test Different Distances**:
```bash
# Test obstacle detected (should stop)
ros2 topic pub --rate 1 /distance_sensor std_msgs/msg/Float32 "data: 0.5"

# Test safe distance (should move)
ros2 topic pub --rate 1 /distance_sensor std_msgs/msg/Float32 "data: 3.0"
```

---

### 2. state_machine_agent.py - Stateful Agent

**Purpose**: Demonstrates state machine pattern with IDLE, MOVING, STOPPED states and hysteresis.

**Concepts Covered**:
- State machine implementation using Python Enum
- State transitions based on sensor input
- Hysteresis to prevent oscillation (resume_distance > safe_distance)
- Separation of state logic from action execution

**How to Run**:

```bash
# Terminal 1: Run the state machine agent
python3 state_machine_agent.py

# Terminal 2: Simulate distance sensor
ros2 topic pub --rate 1 /distance_sensor std_msgs/msg/Float32 "data: 2.0"

# Terminal 3: Monitor velocity and watch for state transitions in Terminal 1
ros2 topic echo /cmd_vel
```

**State Transitions to Observe**:

```bash
# Start in IDLE, transition to MOVING (distance 2.0m > safe_distance 1.0m)
ros2 topic pub --rate 1 /distance_sensor std_msgs/msg/Float32 "data: 2.0"

# Transition to STOPPED (distance 0.8m ≤ safe_distance 1.0m)
ros2 topic pub --rate 1 /distance_sensor std_msgs/msg/Float32 "data: 0.8"

# Remain STOPPED (distance 1.2m < resume_distance 1.5m)
ros2 topic pub --rate 1 /distance_sensor std_msgs/msg/Float32 "data: 1.2"

# Transition to MOVING (distance 1.6m > resume_distance 1.5m)
ros2 topic pub --rate 1 /distance_sensor std_msgs/msg/Float32 "data: 1.6"
```

**Why Hysteresis?** Notice that stopping threshold (1.0m) differs from resume threshold (1.5m). This prevents rapid state switching when distance hovers near the threshold.

---

## Testing Agent Communication

### Verify Topics and Nodes

```bash
# List active nodes
ros2 node list
# Should show: /obstacle_avoidance_agent or /state_machine_agent

# List active topics
ros2 topic list
# Should show: /distance_sensor, /cmd_vel

# Check topic connections
ros2 topic info /distance_sensor
# Should show: 1 publisher, 1 subscriber

ros2 topic info /cmd_vel
# Should show: 1 publisher, 0 subscribers (no robot controller in simulation)
```

### Monitor Message Flow

```bash
# View distance sensor messages
ros2 topic echo /distance_sensor

# View velocity commands
ros2 topic echo /cmd_vel

# Check publish rate
ros2 topic hz /cmd_vel
```

---

## Troubleshooting

### Issue: "No module named 'geometry_msgs'"

**Cause**: ROS 2 message packages not installed

**Solution**:
```bash
sudo apt update
sudo apt install ros-humble-geometry-msgs
source /opt/ros/humble/setup.bash
```

---

### Issue: Agent doesn't respond to distance messages

**Possible Causes**:
1. **Agent not subscribed**: Check `ros2 topic info /distance_sensor` shows 1 subscriber
2. **Wrong message type**: Ensure publishing `std_msgs/msg/Float32` not `std_msgs/msg/Float64`
3. **QoS mismatch**: Publisher and subscriber QoS policies incompatible (rare with default settings)

**Debugging**:
```bash
# Verify agent is receiving messages (check agent terminal for log output)
# Should see: "Distance: X.XXm → Velocity: X.XXm/s"

# Test with ros2 topic echo to verify messages are being published
ros2 topic echo /distance_sensor
```

---

### Issue: State transitions happen too frequently

**Cause**: Distance hovering near threshold without hysteresis

**Solution**: The `state_machine_agent.py` already implements hysteresis (resume_distance = 1.5m > safe_distance = 1.0m). Increase the gap if needed:

```python
self.safe_distance = 1.0
self.resume_distance = 2.0  # Larger gap = more stable
```

---

## Extensions

Try these modifications to deepen your understanding:

1. **Exercise 2.1: Parameterized Safe Distance** (Beginner)
   - Modify agent to read `safe_distance` from ROS 2 parameter
   - Use: `self.declare_parameter('safe_distance', 1.0)`
   - Test: `ros2 run <package> obstacle_avoidance_agent --ros-args -p safe_distance:=1.5`

2. **Exercise 2.2: Multi-Level Speed** (Intermediate)
   - Implement three speed levels:
     - Distance > 2.0m: 0.8 m/s (fast)
     - 1.0m < distance ≤ 2.0m: 0.5 m/s (medium)
     - Distance ≤ 1.0m: 0.0 m/s (stop)

3. **Exercise 2.3: Emergency Stop State** (Advanced)
   - Add EMERGENCY_STOP state for distance < 0.3m
   - Publish negative velocity (-0.2 m/s) to back up
   - Transition to STOPPED when distance > 0.5m

4. **Exercise 2.4: Rotation Behavior** (Advanced)
   - When STOPPED, rotate in place (`velocity.angular.z = 0.5`) to search for clear path
   - Transition to MOVING when distance > safe_distance

---

## Real Robot Integration

To use these agents with a physical robot:

1. **Replace Simulated Sensor**: Subscribe to your robot's actual sensor topic
   ```python
   # Example: LiDAR front distance
   self.subscription = self.create_subscription(
       LaserScan, '/scan', self.lidar_callback, 10)
   ```

2. **Update Decision Logic**: Extract distance from real sensor message format
   ```python
   def lidar_callback(self, msg):
       # Get minimum distance from front 30 degrees of LiDAR
       front_ranges = msg.ranges[len(msg.ranges)//2 - 15 : len(msg.ranges)//2 + 15]
       distance = min(front_ranges)
       # Continue with existing logic...
   ```

3. **Test Safely**: Start with very low velocities (0.1 m/s) and short distances

---

**Code Tested On**: Ubuntu 22.04, ROS 2 Humble, Python 3.10
**Last Updated**: 2025-12-18
**License**: Educational use
