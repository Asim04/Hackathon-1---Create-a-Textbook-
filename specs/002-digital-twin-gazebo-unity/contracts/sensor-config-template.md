# Sensor Configuration Template: Module 2 - The Digital Twin

**Purpose**: Gazebo sensor plugin examples with noise models for educational simulation
**Version**: 1.0.0
**Compatible**: Gazebo Classic 11, ROS 2 Humble gazebo_ros_pkgs
**Target Sensors**: LiDAR (ray), Depth Camera (RGB-D), IMU, Contact

---

## Overview

This template provides ready-to-use Gazebo sensor configurations with:
- Realistic noise models (Gaussian)
- ROS 2 topic publishing
- Performance-optimized parameters for educational use
- Common troubleshooting annotations

**Usage**: Copy sensor blocks into robot URDF `<gazebo>` tags or world SDF files.

---

## Sensor 1: 2D LiDAR (Ray Sensor)

### Use Case
Planar laser scanner for obstacle detection, SLAM, navigation (e.g., Hokuyo URG-04LX, SICK LMS-100)

### Configuration (URDF Gazebo Plugin)

```xml
<!-- Add to robot URDF inside <gazebo reference="lidar_link"> tag -->
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <!-- Update rate (Hz) - Balance data frequency vs CPU load -->
    <update_rate>10.0</update_rate>

    <!-- Visualization in Gazebo GUI -->
    <visualize>true</visualize>

    <!-- Ray configuration -->
    <ray>
      <scan>
        <!-- Horizontal scan parameters -->
        <horizontal>
          <samples>360</samples>          <!-- Number of rays (1° resolution) -->
          <resolution>1</resolution>      <!-- Always 1 for accurate sampling -->
          <min_angle>-3.14159</min_angle> <!-- -180° in radians -->
          <max_angle>3.14159</max_angle>  <!-- +180° in radians -->
        </horizontal>

        <!-- Vertical scan (1 for 2D LiDAR) -->
        <vertical>
          <samples>1</samples>
        </vertical>
      </scan>

      <!-- Range limits (meters) -->
      <range>
        <min>0.5</min>  <!-- Minimum valid distance (deadzone) -->
        <max>10.0</max> <!-- Maximum detection range -->
        <resolution>0.01</resolution> <!-- Distance precision -->
      </range>

      <!-- Gaussian noise model -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>         <!-- No systematic bias -->
        <stddev>0.01</stddev>    <!-- ±1cm standard deviation (realistic) -->
      </noise>
    </ray>

    <!-- Gazebo ROS 2 Plugin -->
    <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
      <!-- ROS 2 namespace (optional) -->
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping> <!-- Publishes to /robot/scan -->
      </ros>

      <!-- Output message type -->
      <output_type>sensor_msgs/LaserScan</output_type>

      <!-- TF frame for sensor data -->
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### ROS 2 Topic Output

```bash
# Check published topic
ros2 topic list | grep scan
# Output: /robot/scan

# View message structure
ros2 topic echo /robot/scan --once
```

**Expected Message**:
```yaml
header:
  frame_id: "lidar_link"
angle_min: -3.14159
angle_max: 3.14159
angle_increment: 0.01745 # 1° in radians
range_min: 0.5
range_max: 10.0
ranges: [5.2, 5.3, 5.1, ...] # 360 floats
```

### Validation Checklist

- [ ] Sensor publishes to correct topic (`/robot/scan`)
- [ ] Range data within [0.5, 10.0] meters
- [ ] 360 range measurements per message
- [ ] Frame rate ~10 Hz (`ros2 topic hz /robot/scan`)
- [ ] Visualizes in RViz (Add > LaserScan > Topic: /robot/scan)

---

## Sensor 2: Depth Camera (RGB-D)

### Use Case
3D perception for object detection, depth estimation (e.g., Intel RealSense D435, Kinect)

### Configuration (URDF Gazebo Plugin)

```xml
<gazebo reference="camera_depth_link">
  <sensor name="depth_camera" type="depth">
    <!-- Update rate (Hz) -->
    <update_rate>30.0</update_rate>

    <!-- Camera intrinsics -->
    <camera>
      <!-- Horizontal field of view (radians) -->
      <horizontal_fov>1.047</horizontal_fov> <!-- 60° -->

      <!-- Image dimensions (pixels) -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format> <!-- RGB color -->
      </image>

      <!-- Clipping planes (meters) -->
      <clip>
        <near>0.1</near>  <!-- Minimum depth -->
        <far>10.0</far>   <!-- Maximum depth -->
      </clip>

      <!-- Gaussian noise (depth measurement) -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev> <!-- ±2cm depth error (realistic for RealSense) -->
      </noise>
    </camera>

    <!-- Gazebo ROS 2 Depth Camera Plugin -->
    <plugin name="gazebo_ros_depth_camera" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/image_raw:=camera/rgb/image_raw</remapping>
        <remapping>~/depth/image_raw:=camera/depth/image_raw</remapping>
        <remapping>~/camera_info:=camera/rgb/camera_info</remapping>
        <remapping>~/depth/camera_info:=camera/depth/camera_info</remapping>
        <remapping>~/points:=camera/depth/points</remapping>
      </ros>

      <!-- Frame names -->
      <frame_name>camera_depth_optical_frame</frame_name>

      <!-- Baseline for stereo (meters, 0 for monocular depth) -->
      <baseline>0.0</baseline>
    </plugin>
  </sensor>
</gazebo>
```

### ROS 2 Topic Output

```bash
# Check published topics
ros2 topic list | grep camera
# Output:
#   /robot/camera/rgb/image_raw (sensor_msgs/Image)
#   /robot/camera/depth/image_raw (sensor_msgs/Image, 16-bit depth)
#   /robot/camera/depth/points (sensor_msgs/PointCloud2)
#   /robot/camera/rgb/camera_info (sensor_msgs/CameraInfo)
```

**Expected Messages**:
- **RGB Image**: 640x480 color image at 30 Hz
- **Depth Image**: 640x480 depth map (millimeters encoded as 16-bit)
- **Point Cloud**: 3D points in camera frame (X, Y, Z coordinates)

### Validation Checklist

- [ ] RGB image displays in RViz (Add > Image > Topic: /robot/camera/rgb/image_raw)
- [ ] Depth image shows grayscale depth map (closer = darker)
- [ ] Point cloud visualizes in RViz (Add > PointCloud2 > Topic: /robot/camera/depth/points)
- [ ] Frame rate ~30 Hz
- [ ] Depth range [0.1, 10.0] meters

---

## Sensor 3: IMU (Inertial Measurement Unit)

### Use Case
Robot orientation, acceleration, angular velocity for state estimation (e.g., BNO055, MPU-9250)

### Configuration (URDF Gazebo Plugin)

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <!-- Update rate (Hz) - High frequency for control loops -->
    <update_rate>100.0</update_rate>

    <!-- IMU plugin configuration -->
    <plugin name="gazebo_ros_imu_sensor" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping> <!-- Publishes to /robot/imu/data -->
      </ros>

      <!-- Frame name -->
      <frame_name>imu_link</frame_name>

      <!-- Initial orientation (quaternion: x, y, z, w) -->
      <initial_orientation_as_reference>false</initial_orientation_as_reference>

      <!-- Noise models for each measurement -->
      <!-- Angular velocity noise (rad/s) -->
      <angular_velocity_stdev>0.01</angular_velocity_stdev> <!-- ±0.01 rad/s -->

      <!-- Linear acceleration noise (m/s²) -->
      <linear_acceleration_stdev>0.05</linear_acceleration_stdev> <!-- ±0.05 m/s² -->

      <!-- Orientation noise (radians) -->
      <orientation_stdev>0.01</orientation_stdev> <!-- ±0.01 rad -->
    </plugin>
  </sensor>
</gazebo>
```

### ROS 2 Topic Output

```bash
# Check published topic
ros2 topic echo /robot/imu/data --once
```

**Expected Message** (sensor_msgs/Imu):
```yaml
header:
  frame_id: "imu_link"
orientation: # Quaternion (x, y, z, w)
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance: [0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001] # Diagonal
angular_velocity: # rad/s
  x: 0.02
  y: -0.01
  z: 0.00
angular_velocity_covariance: [0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001]
linear_acceleration: # m/s²
  x: 0.1
  y: -0.05
  z: 9.81
linear_acceleration_covariance: [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
```

### Validation Checklist

- [ ] IMU publishes to `/robot/imu/data`
- [ ] Orientation quaternion norm ≈ 1.0 (valid quaternion)
- [ ] Linear acceleration Z ≈ 9.81 m/s² when stationary (gravity)
- [ ] Angular velocity near zero when robot not rotating
- [ ] Frame rate ~100 Hz

---

## Sensor 4: Contact Sensor (Bumper)

### Use Case
Collision detection for safety, reactive behaviors (e.g., robot bumper, gripper contact)

### Configuration (URDF Gazebo Plugin)

```xml
<gazebo reference="bumper_link">
  <sensor name="bumper_sensor" type="contact">
    <!-- Contact sensor detects collisions on this link -->
    <contact>
      <collision>bumper_collision</collision> <!-- Must match <collision> name in URDF -->
    </contact>

    <!-- Update rate (Hz) -->
    <update_rate>50.0</update_rate>

    <!-- Gazebo ROS 2 Bumper Plugin -->
    <plugin name="gazebo_ros_bumper" filename="libgazebo_ros_bumper.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=bumper_states</remapping> <!-- Publishes to /robot/bumper_states -->
      </ros>

      <!-- Frame name -->
      <frame_name>bumper_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### ROS 2 Topic Output

```bash
# Check published topic (only publishes during contact)
ros2 topic echo /robot/bumper_states
```

**Expected Message** (gazebo_msgs/ContactsState):
```yaml
header:
  frame_id: "bumper_link"
states:
  - collision1_name: "robot::bumper_link::bumper_collision"
    collision2_name: "world::obstacle::collision"
    wrenches:
      - force: {x: 12.5, y: 0.0, z: 0.0} # Newtons
        torque: {x: 0.0, y: 0.0, z: 0.0} # Newton-meters
    contact_positions: [{x: 0.5, y: 0.0, z: 0.3}] # Contact point in world frame
    contact_normals: [{x: 1.0, y: 0.0, z: 0.0}]   # Surface normal
    depths: [0.01] # Penetration depth (meters)
```

### Validation Checklist

- [ ] Topic silent when no contact
- [ ] Publishes immediately upon collision
- [ ] Force magnitude increases with impact velocity
- [ ] Contact position accurately reflects collision point

---

## Noise Model Guidelines

### Gaussian Noise Parameters

| Sensor | Parameter | Typical Range | Rationale |
|--------|-----------|---------------|-----------|
| **LiDAR** | stddev (distance) | 0.005-0.02 m | High-quality: 5mm, Consumer: 2cm |
| **Depth Camera** | stddev (depth) | 0.01-0.05 m | RealSense D435: ~2cm at 3m distance |
| **IMU (gyro)** | angular_velocity_stdev | 0.001-0.01 rad/s | MEMS: 0.01 rad/s, Fiber optic: 0.001 rad/s |
| **IMU (accel)** | linear_acceleration_stdev | 0.01-0.1 m/s² | MEMS: 0.05 m/s², High-grade: 0.01 m/s² |

### When to Adjust Noise

- **Educational Demos**: Use LOW noise (0.5× typical) to emphasize concepts over error handling
- **Realistic Testing**: Use TYPICAL noise (1× values above) to match real-world sensors
- **Stress Testing**: Use HIGH noise (2× typical) to validate robustness

---

## Performance Optimization

### Update Rate Guidelines

| Sensor | Recommended Hz | Reasoning |
|--------|----------------|-----------|
| LiDAR | 10-20 | Sufficient for navigation, lower CPU load |
| Depth Camera (RGB) | 15-30 | Standard video rates, balance quality/bandwidth |
| Depth Camera (Point Cloud) | 5-10 | High data volume (307k points), reduce for performance |
| IMU | 50-200 | High frequency needed for control loops |
| Contact | 50-100 | Fast response for collision detection |

### Gazebo Performance Tips

- **Reduce Ray Samples**: LiDAR with 180 samples (2° resolution) instead of 360 for 2× speedup
- **Lower Camera Resolution**: 320×240 for depth cameras in non-vision tasks (4× faster)
- **Disable Visualization**: Set `<visualize>false</visualize>` for sensors not being debugged

---

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Sensor not publishing | Plugin filename incorrect | Verify `libgazebo_ros_*.so` exists: `ls /opt/ros/humble/lib/libgazebo_ros_*` |
| High CPU usage | Update rate too high | Reduce to minimum needed (e.g., LiDAR 10 Hz instead of 50 Hz) |
| Noisy depth image | stddev too high | Lower `<noise><stddev>` to 0.01-0.02 m |
| IMU orientation wrong | Frame convention mismatch | Check `<initial_orientation_as_reference>true</initial_orientation_as_reference>` |
| Contact sensor always triggers | Collision geometry too large | Shrink `<collision>` mesh or increase `<contact_surface_layer>` in physics |

---

## Integration with Unity

To visualize Gazebo sensor data in Unity:

1. **LiDAR**: Subscribe to `/robot/scan` in Unity, instantiate line renderers for each ray
2. **Depth Camera**: Subscribe to `/robot/camera/depth/image_raw`, convert to Unity Texture2D
3. **IMU**: Subscribe to `/robot/imu/data`, apply orientation quaternion to Unity GameObject transform
4. **Contact**: Subscribe to `/robot/bumper_states`, trigger particle effects or sounds on collision

**Example** (Unity C# LiDAR Visualizer):
```csharp
ROSConnection.GetOrCreateInstance().Subscribe<LaserScanMsg>(
    "/robot/scan",
    (msg) => {
        for (int i = 0; i < msg.ranges.Length; i++) {
            float angle = msg.angle_min + i * msg.angle_increment;
            float range = msg.ranges[i];
            Vector3 point = new Vector3(
                range * Mathf.Cos(angle),
                0,
                range * Mathf.Sin(angle)
            );
            // Render point in Unity scene
        }
    }
);
```

---

## Testing and Validation

### Manual Testing Procedure

1. **Launch Gazebo World**:
   ```bash
   ros2 launch your_robot_description gazebo.launch.py
   ```

2. **Verify Topic Publishing**:
   ```bash
   ros2 topic list # Check all sensor topics present
   ros2 topic hz /robot/scan # Verify update rate
   ```

3. **Visualize in RViz**:
   ```bash
   ros2 run rviz2 rviz2
   # Add > LaserScan, Image, PointCloud2, Imu, etc.
   ```

4. **Check Data Quality**:
   ```bash
   ros2 topic echo /robot/scan --once | grep range_min # Should match config
   ```

### Automated Validation (pytest)

```python
def test_lidar_publishing():
    """Verify LiDAR publishes at correct rate with valid data"""
    import rclpy
    from sensor_msgs.msg import LaserScan

    node = rclpy.create_node('test_node')
    msgs = []

    def callback(msg):
        msgs.append(msg)
        assert len(msg.ranges) == 360, "Wrong number of rays"
        assert all(0.5 <= r <= 10.0 or r == float('inf') for r in msg.ranges), "Invalid range data"

    sub = node.create_subscription(LaserScan, '/robot/scan', callback, 10)

    # Collect 10 messages
    while len(msgs) < 10:
        rclpy.spin_once(node, timeout_sec=0.5)

    # Check rate (should be ~10 Hz)
    time_diffs = [msgs[i+1].header.stamp.sec - msgs[i].header.stamp.sec for i in range(len(msgs)-1)]
    avg_period = sum(time_diffs) / len(time_diffs)
    assert 0.08 < avg_period < 0.12, f"Rate not ~10 Hz (period: {avg_period}s)"
```

---

**Template Version**: 1.0.0
**Last Updated**: 2025-12-18
**Gazebo Plugins Used**:
- `libgazebo_ros_ray_sensor.so` (LiDAR)
- `libgazebo_ros_camera.so` (RGB-D)
- `libgazebo_ros_imu_sensor.so` (IMU)
- `libgazebo_ros_bumper.so` (Contact)
**ROS 2 Message Types**: LaserScan, Image, CameraInfo, PointCloud2, Imu, ContactsState
