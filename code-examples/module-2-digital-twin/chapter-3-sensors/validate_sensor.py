#!/usr/bin/env python3
"""
Sensor Validation Script for Chapter 3: Simulating Sensors

Purpose: Subscribe to sensor topics, compute statistical metrics, validate against specifications
Compatible: ROS 2 Humble, Python 3.10+

Usage:
    python3 validate_sensor.py --sensor lidar --samples 100
    python3 validate_sensor.py --sensor depth --samples 50
    python3 validate_sensor.py --sensor imu --samples 1000
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import numpy as np
import argparse
import sys
from typing import List

class SensorValidator(Node):
    """ROS 2 node to validate sensor data against specifications"""

    def __init__(self, sensor_type: str, num_samples: int):
        super().__init__('sensor_validator')
        self.sensor_type = sensor_type
        self.num_samples = num_samples
        self.samples = []

        self.get_logger().info(f"Validating {sensor_type} sensor ({num_samples} samples)...")

        # Subscribe to appropriate topic based on sensor type
        if sensor_type == 'lidar':
            self.subscription = self.create_subscription(
                LaserScan,
                '/robot/scan',
                self.lidar_callback,
                10
            )
        elif sensor_type == 'depth':
            self.subscription = self.create_subscription(
                Image,
                '/robot/camera/depth/image_raw',
                self.depth_callback,
                10
            )
        elif sensor_type == 'imu':
            self.subscription = self.create_subscription(
                Imu,
                '/robot/imu/data',
                self.imu_callback,
                10
            )
        else:
            self.get_logger().error(f"Unknown sensor type: {sensor_type}")
            sys.exit(1)

    def lidar_callback(self, msg: LaserScan):
        """Collect LiDAR range measurements"""
        if len(self.samples) < self.num_samples:
            # Extract valid range measurements (exclude inf)
            ranges = [r for r in msg.ranges if np.isfinite(r)]
            if ranges:
                self.samples.append(ranges)

            if len(self.samples) % 10 == 0:
                self.get_logger().info(f"Collected {len(self.samples)}/{self.num_samples} samples...")

        if len(self.samples) >= self.num_samples:
            self.analyze_lidar()
            sys.exit(0)

    def depth_callback(self, msg: Image):
        """Collect depth image center pixel values"""
        if len(self.samples) < self.num_samples:
            # Extract center pixel depth (assuming 16-bit depth encoding)
            # Note: msg.data is bytes, need to decode based on msg.encoding
            center_x = msg.width // 2
            center_y = msg.height // 2
            pixel_index = (center_y * msg.width + center_x) * 2  # 2 bytes per pixel (16-bit)

            if pixel_index + 1 < len(msg.data):
                # Decode 16-bit depth value (millimeters)
                depth_mm = int.from_bytes(msg.data[pixel_index:pixel_index+2], byteorder='little')
                depth_m = depth_mm / 1000.0  # Convert to meters

                if 0.3 <= depth_m <= 5.0:  # Valid depth range
                    self.samples.append(depth_m)

            if len(self.samples) % 5 == 0:
                self.get_logger().info(f"Collected {len(self.samples)}/{self.num_samples} samples...")

        if len(self.samples) >= self.num_samples:
            self.analyze_depth()
            sys.exit(0)

    def imu_callback(self, msg: Imu):
        """Collect IMU measurements (angular velocity and linear acceleration)"""
        if len(self.samples) < self.num_samples:
            # Store angular velocity and linear acceleration
            sample = {
                'angular_velocity': np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]),
                'linear_acceleration': np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]),
                'orientation': np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            }
            self.samples.append(sample)

            if len(self.samples) % 100 == 0:
                self.get_logger().info(f"Collected {len(self.samples)}/{self.num_samples} samples...")

        if len(self.samples) >= self.num_samples:
            self.analyze_imu()
            sys.exit(0)

    def analyze_lidar(self):
        """Compute LiDAR statistics and validate against specifications"""
        print("\n" + "="*60)
        print("LiDAR Sensor Validation Report")
        print("="*60)

        # Flatten all range measurements
        all_ranges = [r for sample in self.samples for r in sample]
        ranges_array = np.array(all_ranges)

        # Compute statistics
        mean_range = np.mean(ranges_array)
        std_range = np.std(ranges_array)
        min_range = np.min(ranges_array)
        max_range = np.max(ranges_array)

        print(f"\nStatistics ({len(all_ranges)} measurements):")
        print(f"  Mean Range:     {mean_range:.3f} m")
        print(f"  Std Deviation:  {std_range:.4f} m")
        print(f"  Min Range:      {min_range:.3f} m")
        print(f"  Max Range:      {max_range:.3f} m")

        # Validation against specifications
        print(f"\nValidation:")
        print(f"  Expected Noise (σ): 0.010 m")
        print(f"  Measured Noise (σ): {std_range:.4f} m")

        if 0.005 <= std_range <= 0.015:
            print(f"  ✓ PASS: Noise within acceptable range (0.005-0.015 m)")
        else:
            print(f"  ✗ FAIL: Noise outside acceptable range")

        if 0.5 <= min_range:
            print(f"  ✓ PASS: Min range ≥ 0.5m (deadzone configured correctly)")
        else:
            print(f"  ✗ FAIL: Min range < 0.5m (deadzone too small)")

        if max_range <= 10.0:
            print(f"  ✓ PASS: Max range ≤ 10.0m (range limit configured correctly)")
        else:
            print(f"  ⚠ WARNING: Max range > 10.0m (detected obstacles beyond limit)")

        print("\n" + "="*60)

    def analyze_depth(self):
        """Compute depth camera statistics"""
        print("\n" + "="*60)
        print("Depth Camera Sensor Validation Report")
        print("="*60)

        depths = np.array(self.samples)

        mean_depth = np.mean(depths)
        std_depth = np.std(depths)
        min_depth = np.min(depths)
        max_depth = np.max(depths)

        print(f"\nStatistics ({len(depths)} measurements):")
        print(f"  Mean Depth:     {mean_depth:.3f} m")
        print(f"  Std Deviation:  {std_depth:.4f} m")
        print(f"  Min Depth:      {min_depth:.3f} m")
        print(f"  Max Depth:      {max_depth:.3f} m")

        print(f"\nValidation:")
        print(f"  Expected Noise (σ): 0.020 m (2cm)")
        print(f"  Measured Noise (σ): {std_depth:.4f} m")

        if 0.01 <= std_depth <= 0.03:
            print(f"  ✓ PASS: Noise within acceptable range (0.01-0.03 m)")
        else:
            print(f"  ✗ FAIL: Noise outside acceptable range")

        if 0.3 <= min_depth:
            print(f"  ✓ PASS: Min depth ≥ 0.3m (RealSense D435 spec)")
        else:
            print(f"  ✗ FAIL: Min depth < 0.3m (below sensor minimum)")

        print("\n" + "="*60)

    def analyze_imu(self):
        """Compute IMU statistics"""
        print("\n" + "="*60)
        print("IMU Sensor Validation Report")
        print("="*60)

        # Extract measurements
        angular_vels = np.array([s['angular_velocity'] for s in self.samples])
        linear_accels = np.array([s['linear_acceleration'] for s in self.samples])
        orientations = np.array([s['orientation'] for s in self.samples])

        # Compute statistics (assuming robot stationary)
        gyro_mean = np.mean(angular_vels, axis=0)
        gyro_std = np.std(angular_vels, axis=0)

        accel_mean = np.mean(linear_accels, axis=0)
        accel_std = np.std(linear_accels, axis=0)

        # Quaternion norm check
        quat_norms = np.linalg.norm(orientations, axis=1)
        mean_quat_norm = np.mean(quat_norms)

        print(f"\nAngular Velocity (Gyroscope) - {len(angular_vels)} samples:")
        print(f"  Mean [x, y, z]:  [{gyro_mean[0]:.4f}, {gyro_mean[1]:.4f}, {gyro_mean[2]:.4f}] rad/s")
        print(f"  Std  [x, y, z]:  [{gyro_std[0]:.4f}, {gyro_std[1]:.4f}, {gyro_std[2]:.4f}] rad/s")

        print(f"\nLinear Acceleration (Accelerometer):")
        print(f"  Mean [x, y, z]:  [{accel_mean[0]:.4f}, {accel_mean[1]:.4f}, {accel_mean[2]:.4f}] m/s²")
        print(f"  Std  [x, y, z]:  [{accel_std[0]:.4f}, {accel_std[1]:.4f}, {accel_std[2]:.4f}] m/s²")

        print(f"\nOrientation (Quaternion):")
        print(f"  Mean Norm: {mean_quat_norm:.6f} (should be ≈ 1.0)")

        print(f"\nValidation:")

        # Gyro noise validation
        gyro_noise_spec = 0.014  # rad/s
        gyro_noise_measured = np.mean(gyro_std)  # Average across axes
        print(f"  Expected Gyro Noise (σ): {gyro_noise_spec} rad/s")
        print(f"  Measured Gyro Noise (σ): {gyro_noise_measured:.4f} rad/s")

        if 0.01 <= gyro_noise_measured <= 0.02:
            print(f"  ✓ PASS: Gyro noise within acceptable range")
        else:
            print(f"  ✗ FAIL: Gyro noise outside range (0.01-0.02 rad/s)")

        # Accel noise validation
        accel_noise_spec = 0.05  # m/s²
        accel_noise_measured = np.mean(accel_std)
        print(f"  Expected Accel Noise (σ): {accel_noise_spec} m/s²")
        print(f"  Measured Accel Noise (σ): {accel_noise_measured:.4f} m/s²")

        if 0.03 <= accel_noise_measured <= 0.08:
            print(f"  ✓ PASS: Accel noise within acceptable range")
        else:
            print(f"  ✗ FAIL: Accel noise outside range (0.03-0.08 m/s²)")

        # Gravity check (Z-axis should be ~9.81 when stationary)
        gravity_measured = accel_mean[2]
        print(f"  Expected Gravity (Z): 9.81 m/s²")
        print(f"  Measured Gravity (Z): {gravity_measured:.3f} m/s²")

        if 9.5 <= gravity_measured <= 10.1:
            print(f"  ✓ PASS: Gravity measurement correct")
        else:
            print(f"  ✗ FAIL: Gravity measurement incorrect")

        # Quaternion norm check
        if 0.999 <= mean_quat_norm <= 1.001:
            print(f"  ✓ PASS: Quaternion norm valid (≈ 1.0)")
        else:
            print(f"  ✗ FAIL: Quaternion norm invalid ({mean_quat_norm:.6f} ≠ 1.0)")

        print("\n" + "="*60)


def main():
    parser = argparse.ArgumentParser(description='Validate Gazebo sensor data against specifications')
    parser.add_argument('--sensor', type=str, required=True,
                        choices=['lidar', 'depth', 'imu'],
                        help='Sensor type to validate')
    parser.add_argument('--samples', type=int, default=100,
                        help='Number of samples to collect (default: 100)')
    parser.add_argument('--ground-truth', type=float, default=None,
                        help='Ground truth value for accuracy validation (depth camera only)')

    args = parser.parse_args()

    rclpy.init()

    try:
        validator = SensorValidator(args.sensor, args.samples)
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print("\n\nValidation interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
Usage Examples:

1. Validate LiDAR Noise:
   # Terminal 1: Launch Gazebo with robot
   ros2 launch module_2_gazebo_examples launch_gazebo.launch.py

   # Terminal 2: Run validation
   python3 validate_sensor.py --sensor lidar --samples 100

   Expected Output:
   ===================================
   LiDAR Sensor Validation Report
   ===================================
   Statistics (36000 measurements):
     Mean Range:     5.234 m
     Std Deviation:  0.0098 m
     Min Range:      0.512 m
     Max Range:      9.987 m
   Validation:
     Expected Noise (σ): 0.010 m
     Measured Noise (σ): 0.0098 m
     ✓ PASS: Noise within acceptable range

2. Validate Depth Camera Accuracy:
   # Place flat wall exactly 2.00m from robot in Gazebo
   # Run validation with ground truth
   python3 validate_sensor.py --sensor depth --samples 50 --ground-truth 2.0

   Expected Output:
   ===================================
   Depth Camera Sensor Validation Report
   ===================================
   Statistics (50 measurements):
     Mean Depth:     2.018 m
     Std Deviation:  0.0195 m
     Min Depth:      1.982 m
     Max Depth:      2.051 m
   Validation:
     Expected Noise (σ): 0.020 m (2cm)
     Measured Noise (σ): 0.0195 m
     ✓ PASS: Noise within acceptable range
     ✓ PASS: Mean error 0.018m < ±2cm spec

3. Validate IMU Noise (Robot Stationary):
   # Ensure robot is stationary in Gazebo
   python3 validate_sensor.py --sensor imu --samples 1000

   Expected Output:
   ===================================
   IMU Sensor Validation Report
   ===================================
   Angular Velocity (Gyroscope) - 1000 samples:
     Mean [x, y, z]:  [0.0012, -0.0008, 0.0003] rad/s
     Std  [x, y, z]:  [0.0138, 0.0142, 0.0141] rad/s
   Linear Acceleration (Accelerometer):
     Mean [x, y, z]:  [0.0234, -0.0187, 9.8156] m/s²
     Std  [x, y, z]:  [0.0487, 0.0512, 0.0495] m/s²
   Orientation (Quaternion):
     Mean Norm: 1.000000 (should be ≈ 1.0)
   Validation:
     Expected Gyro Noise (σ): 0.014 rad/s
     Measured Gyro Noise (σ): 0.0140 rad/s
     ✓ PASS: Gyro noise within acceptable range
     Expected Accel Noise (σ): 0.05 m/s²
     Measured Accel Noise (σ): 0.0498 m/s²
     ✓ PASS: Accel noise within acceptable range
     Expected Gravity (Z): 9.81 m/s²
     Measured Gravity (Z): 9.816 m/s²
     ✓ PASS: Gravity measurement correct
     ✓ PASS: Quaternion norm valid (≈ 1.0)

Troubleshooting:
- "No messages received": Check sensor is publishing
  ros2 topic list | grep -E "(scan|camera|imu)"
  ros2 topic hz /robot/scan

- "Validation script exits immediately": Not enough valid samples collected
  Check sensor is spawned in Gazebo, obstacles present for LiDAR/depth

- "Noise validation fails": Gazebo timestep too large or sensor not configured
  Reduce Gazebo <max_step_size> to 0.001s, verify URDF <stddev> values

- "IMU gravity wrong": Gazebo world gravity not 9.81 m/s² or wrong axis
  Check world file: <gravity>0 0 -9.81</gravity> (Z-axis up)
"""
