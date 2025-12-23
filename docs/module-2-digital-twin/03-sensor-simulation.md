---
sidebar_position: 4
title: "Chapter 3: Simulating Sensors"
description: "Simulate LiDAR, depth cameras, and IMUs in Gazebo with realistic noise models for perception algorithm testing"
---

# Chapter 3: Simulating Sensors

**Module**: The Digital Twin (Gazebo & Unity)
**Estimated Reading Time**: 25 minutes
**Prerequisites**: Chapter 1 (Gazebo Physics), Chapter 2 (Unity Visualization), basic signal processing (Gaussian distributions)

---

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure Gazebo sensor plugins (LiDAR, depth camera, IMU) with realistic parameters
- Apply Gaussian noise models matching real-world sensor datasheets
- Validate simulated sensor accuracy using statistical analysis
- Visualize sensor data in RViz for debugging perception algorithms

---

## Prerequisites

**Knowledge Prerequisites**:
- Gazebo world configuration (Chapter 1)
- URDF robot modeling (Module 1, Chapter 2)
- Basic probability: Gaussian distribution (mean μ, standard deviation σ)

**Software Prerequisites**:
- RViz2 for sensor visualization (`ros2 run rviz2 rviz2`)
- PlotJuggler for time-series analysis (optional): `sudo apt install ros-humble-plotjuggler-ros`

---

## Introduction

Perception algorithms (SLAM, object detection, depth estimation) require realistic sensor data for validation before hardware deployment. Simulating sensors in Gazebo provides ground-truth comparisons: you know exact robot pose, object positions, and can systematically test edge cases (occlusion, lighting changes, sensor noise).

Gazebo sensor plugins model physics-based measurement processes. LiDAR uses ray-casting to compute range, depth cameras render RGB-D images, and IMUs integrate angular velocity with gravity. Adding Gaussian noise approximates real-world imperfections: laser jitter, depth quantization error, IMU bias drift (Liang et al., 2020).

In this chapter, you'll add three core sensors to the humanoid robot, tune noise parameters to match datasheets (Velodyne VLP-16, Intel RealSense D435, Bosch BMI088), and validate accuracy within ±2cm for depth, ±0.01 rad for IMU orientation.

---

## Section 1: Sensor Types in Robotics

### LiDAR (Light Detection and Ranging)

**Measurement**: Time-of-flight distance via laser pulses
**Output**: `sensor_msgs/LaserScan` (2D) or `PointCloud2` (3D)
**Use Cases**: SLAM, obstacle avoidance, localization

**Key Parameters**: Range 0.5-100m, Resolution 0.01m, Update 5-20 Hz, FOV 360°

**Gazebo Plugin**: `libgazebo_ros_ray_sensor.so`

### Depth Camera (RGB-D)

**Measurement**: Per-pixel depth via stereo or structured light
**Output**: `sensor_msgs/Image` (RGB + depth), `PointCloud2`
**Use Cases**: Object detection, grasping, 3D reconstruction

**Key Parameters**: Resolution 640×480, Depth 0.3-10m, Frame rate 30-90 FPS, FOV 60-90°

**Gazebo Plugin**: `libgazebo_ros_camera.so`

### IMU (Inertial Measurement Unit)

**Measurement**: Angular velocity (gyro), linear acceleration (accel)
**Output**: `sensor_msgs/Imu` (orientation, angular velocity, acceleration)
**Use Cases**: Attitude estimation, dead reckoning, sensor fusion

**Key Parameters**: Update 100-1000 Hz, Gyro noise 0.001-0.01 rad/s, Accel noise 0.01-0.1 m/s²

**Gazebo Plugin**: `libgazebo_ros_imu_sensor.so`

---

## Section 2: Gazebo Sensor Plugins

### LiDAR Plugin Configuration

Add to robot URDF within `<gazebo reference="lidar_link">` tag:

```xml
<sensor name="lidar" type="ray">
  <update_rate>10.0</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.5</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
    <ros><remapping>~/out:=scan</remapping></ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

**Key Settings**:
- `samples`: Number of rays (360 = 1° resolution)
- `stddev`: Gaussian noise (0.01m matches Velodyne VLP-16)
- `update_rate`: Publishing frequency (10 Hz)

### Depth Camera Plugin

```xml
<sensor name="depth_camera" type="depth">
  <update_rate>30.0</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60° -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip><near>0.3</near><far>5.0</far></clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev> <!-- 2cm error -->
    </noise>
  </camera>
  <plugin name="depth_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <remapping>~/image_raw:=camera/rgb/image_raw</remapping>
      <remapping>~/depth/image_raw:=camera/depth/image_raw</remapping>
      <remapping>~/points:=camera/depth/points</remapping>
    </ros>
    <frame_name>camera_depth_optical_frame</frame_name>
  </plugin>
</sensor>
```

**Key Settings**:
- `stddev`: 0.02m (RealSense D435 spec)
- `clip`: Depth range validation
- Outputs: RGB, depth (16-bit), point cloud

### IMU Plugin

```xml
<sensor name="imu" type="imu">
  <update_rate>100.0</update_rate>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros><remapping>~/out:=imu/data</remapping></ros>
    <frame_name>imu_link</frame_name>
    <angular_velocity_stdev>0.014</angular_velocity_stdev> <!-- rad/s -->
    <linear_acceleration_stdev>0.05</linear_acceleration_stdev> <!-- m/s² -->
    <orientation_stdev>0.01</orientation_stdev> <!-- rad -->
  </plugin>
</sensor>
```

**Key Settings**:
- `angular_velocity_stdev`: 0.014 rad/s matches Bosch BMI088 gyroscope
- `linear_acceleration_stdev`: 0.05 m/s² typical for MEMS accelerometers
- High update rate (100 Hz) needed for control loops

---

## Section 3: Sensor Noise Models

### Gaussian Noise

Most sensor errors follow Gaussian distribution: `measurement = true_value + N(μ, σ²)`

**Parameters**:
- **Mean (μ)**: Systematic bias (usually 0 for calibrated sensors)
- **Standard Deviation (σ)**: Noise magnitude, determines precision

**Example**: LiDAR with σ=0.01m means 68% of measurements within ±1cm, 95% within ±2cm (assuming μ=0).

### Sensor-Specific Noise Characteristics

**LiDAR Noise Sources**:
- Pulse timing jitter: ±0.5-2cm
- Ambient light interference: Increases with sunlight
- Surface reflectivity: Dark/glossy surfaces reduce SNR

**Realistic Values**: σ = 0.005-0.02m (Velodyne: 0.01m, consumer: 0.02-0.03m per manufacturer datasheets)

**Depth Camera Noise**:
- Quantization error: Depth resolution (e.g., 1mm steps)
- Infrared speckle noise: Structured light randomness
- Distance-dependent: Error grows with range (σ ∝ d²)

**Realistic Values**: σ = 0.01m at 1m, 0.04m at 3m per Intel datasheet specifications (Intel Corporation, 2019)

**IMU Noise**:
- **Gyroscope**: Angular random walk (ARW), bias instability
- **Accelerometer**: Velocity random walk (VRW), temperature drift
- **Allan Variance**: Characterizes noise vs averaging time

**Realistic Values**:
- Gyro: σ = 0.001-0.02 rad/s (consumer: 0.01, tactical: 0.001)
- Accel: σ = 0.01-0.1 m/s² (MEMS: 0.05, high-grade: 0.01)

### Matching Datasheets

**Example**: Bosch BMI088 datasheet (Bosch Sensortec, 2018) specifies:
- Gyro noise: 0.014 rad/s/√Hz at 100 Hz → `angular_velocity_stdev=0.014`
- Accel noise: 150 μg/√Hz ≈ 0.05 m/s²/√Hz → `linear_acceleration_stdev=0.05`

Convert noise density (per √Hz) to standard deviation:
σ = noise_density × √(update_rate)

---

## Section 4: Validating Sensor Data

### Validation Metrics

**Accuracy**: Mean error from ground truth
**Precision**: Standard deviation of errors (repeatability)
**Resolution**: Smallest detectable change

**Example**: LiDAR measuring 5.00m target:
- Measured values: [4.99, 5.01, 5.00, 4.98, 5.02]
- Mean error (accuracy): (4.99+5.01+5.00+4.98+5.02)/5 - 5.00 = 0.00m ✓
- Std dev (precision): 0.015m ✓

### Statistical Analysis

**Collect Data**:
```bash
ros2 topic echo /scan --once > lidar_sample.yaml
```

**Compute Statistics** (Python):
```python
import numpy as np
ranges = np.array([4.99, 5.01, 5.00, 4.98, 5.02])
mean_error = np.mean(ranges) - 5.00  # Accuracy
std_dev = np.std(ranges)              # Precision
print(f"Accuracy: {mean_error:.3f}m, Precision: {std_dev:.3f}m")
```

### Visualization with RViz

**LiDAR**:
```bash
ros2 run rviz2 rviz2
# Add > LaserScan > Topic: /scan
# Visual: Red rays showing detected obstacles
```

**Depth Camera**:
```bash
# Add > Image > Topic: /camera/depth/image_raw
# Add > PointCloud2 > Topic: /camera/depth/points
# Visual: Grayscale depth image + 3D point cloud
```

**IMU**:
```bash
# Add > Imu > Topic: /imu/data
# Visual: Orientation arrow, acceleration vector
```

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| LiDAR returns all `inf` | No obstacles in range | Check `<min>` and `<max>` cover expected distances |
| Depth image is black | Camera clipping planes wrong | Adjust `<near>` and `<far>` to scene depth |
| IMU orientation drifts | Gyro bias accumulation | Use sensor fusion (robot_localization package) |
| High noise (σ > expected) | Gazebo timestep too large | Reduce `<max_step_size>` to 0.001s |

---

## Practice Exercises

### Exercise 3.1: Add LiDAR to Robot (Beginner)
**Objective**: Attach LiDAR sensor to robot, visualize scan in RViz

**Steps**:
1. Copy `lidar_sensor.urdf.xacro` to robot description package
2. Include in main URDF: `<xacro:include filename="lidar_sensor.urdf.xacro"/>`
3. Launch Gazebo: `ros2 launch your_package gazebo.launch.py`
4. Open RViz: Add LaserScan display, topic `/scan`

**Expected**: Red rays emanating from robot showing 360° obstacle detection

**Estimated Time**: 20 minutes

---

### Exercise 3.2: Configure IMU Noise (Intermediate)
**Objective**: Match IMU plugin noise to Bosch BMI088 datasheet

**Datasheet Values**:
- Gyro: 0.014 rad/s/√Hz at 100 Hz
- Accel: 0.05 m/s²/√Hz at 100 Hz

**Steps**:
1. Edit `imu_sensor.urdf.xacro`
2. Set `angular_velocity_stdev="0.014"`
3. Set `linear_acceleration_stdev="0.05"`
4. Record 1000 IMU samples: `ros2 topic echo /imu/data --once`
5. Compute actual σ with validation script

**Expected**: Measured σ within ±10% of configured values

**Estimated Time**: 45 minutes

---

### Exercise 3.3: Validate Depth Accuracy (Advanced)
**Objective**: Measure depth camera accuracy against ground truth

**Setup**:
1. Place flat wall 2.00m from robot in Gazebo
2. Record depth image center pixel
3. Extract depth value, compare to known distance

**Validation**:
```python
depth_samples = [1.98, 2.01, 2.00, 1.99, 2.02]  # meters
mean_error = abs(np.mean(depth_samples) - 2.00)
assert mean_error < 0.02, f"Accuracy {mean_error:.3f}m exceeds ±2cm spec"
```

**Expected**: Mean error \<2cm, matching `\<stddev>0.02\</stddev>` configuration

**Estimated Time**: 60 minutes

---

## Summary

This chapter covered:
1. **Sensor types**: LiDAR (ray-casting, LaserScan), depth camera (RGB-D, PointCloud2), IMU (orientation, angular velocity, acceleration)
2. **Gazebo plugins**: `libgazebo_ros_ray_sensor`, `libgazebo_ros_camera`, `libgazebo_ros_imu_sensor` with parameters (update_rate, noise models, topic remapping)
3. **Noise models**: Gaussian distribution (mean μ, stddev σ), matching datasheets (Velodyne: σ=0.01m, RealSense: σ=0.02m, BMI088: σ=0.014 rad/s)
4. **Validation**: Accuracy (mean error), precision (std dev), RViz visualization, statistical analysis with Python/PlotJuggler
5. **Troubleshooting**: Clipping planes, noise magnitude, sensor fusion for IMU drift correction

**Forward Reference**: In Chapter 4, you'll integrate these sensors into a complete digital twin lab, synchronizing Gazebo physics + Unity visualization + sensor streams + control commands for end-to-end robot testing.

---

## Further Reading

- **Liang, J., et al. (2020)**. Crowd-steer: Realtime smooth and collision-free robot navigation in densely crowded scenarios trained using high-fidelity simulation. *arXiv preprint arXiv:2004.03408*. - LiDAR simulation for crowd navigation algorithms
- **Intel Corporation. (2019)**. *Intel RealSense D400 Series Datasheet*. Retrieved from https://www.intelrealsense.com/depth-camera-d435/ - Depth camera specifications (range, resolution, noise characteristics)
- **Bosch Sensortec. (2018)**. *BMI088 6-axis Motion Tracking IMU Datasheet*. Retrieved from https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/ - IMU noise density specifications for gyroscope and accelerometer

---

**Next Chapter**: [Chapter 4: Lab - Building a Digital Twin](./lab-digital-twin)
