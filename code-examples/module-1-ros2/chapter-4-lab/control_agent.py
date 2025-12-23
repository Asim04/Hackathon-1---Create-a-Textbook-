#!/usr/bin/env python3
"""
Lab Control Agent
Purpose: Command arm movements based on obstacle detection
Prerequisites: ROS 2 Humble, Python 3.10+
Usage: python3 control_agent.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math


class ControlAgent(Node):
    """
    Control agent for humanoid arm control based on obstacle detection.

    Subscribes to:
        /obstacle_detected (String): Detection message from perception agent

    Publishes to:
        /joint_commands (JointState): Joint position commands for robot arms

    Control Strategy:
    - No obstacle: Arms at neutral position (shoulder 0°, elbow 90°)
    - Left obstacle: Raise left arm (shoulder -45°) to avoid
    - Right obstacle: Raise right arm (shoulder 45°) to avoid
    - Both obstacles: Raise both arms

    Key Concepts:
    - Reactive control (immediate response to perception)
    - Joint position commanding using sensor_msgs/JointState
    - Coordinate transformation (degrees → radians)
    """

    def __init__(self):
        super().__init__('control_agent')

        # Subscriber for obstacle detection
        self.detection_sub = self.create_subscription(
            String,
            '/obstacle_detected',
            self.detection_callback,
            10
        )

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        # Joint position targets (radians)
        self.neutral_shoulder = 0.0  # Arms at side
        self.raised_shoulder = math.radians(-45)  # Arms raised to avoid obstacle
        self.elbow_angle = math.radians(90)  # Elbow bent 90 degrees

        self.get_logger().info('Control Agent started')
        self.get_logger().info(f'Neutral shoulder: {math.degrees(self.neutral_shoulder):.1f}°')
        self.get_logger().info(f'Raised shoulder: {math.degrees(self.raised_shoulder):.1f}°')
        self.get_logger().info(f'Elbow angle: {math.degrees(self.elbow_angle):.1f}°')

    def detection_callback(self, msg):
        """
        Process obstacle detection and command arm movements.

        Implements reactive control strategy based on detection message.

        Args:
            msg (std_msgs.msg.String): Detection message ("left", "right", "both", "none")
        """
        detection = msg.data

        # Determine target joint positions based on detection
        if detection == "left":
            # Left obstacle detected → raise left arm only
            left_shoulder = self.raised_shoulder
            right_shoulder = self.neutral_shoulder
        elif detection == "right":
            # Right obstacle detected → raise right arm only
            left_shoulder = self.neutral_shoulder
            right_shoulder = self.raised_shoulder
        elif detection == "both":
            # Both obstacles detected → raise both arms
            left_shoulder = self.raised_shoulder
            right_shoulder = self.raised_shoulder
        else:  # "none"
            # No obstacles → both arms at neutral
            left_shoulder = self.neutral_shoulder
            right_shoulder = self.neutral_shoulder

        # Create JointState message
        joint_cmd = JointState()

        # Set timestamp (important for time-synchronized systems)
        joint_cmd.header.stamp = self.get_clock().now().to_msg()

        # Define joint names (must match URDF joint names exactly)
        joint_cmd.name = [
            'left_shoulder_pitch',
            'left_elbow',
            'right_shoulder_pitch',
            'right_elbow'
        ]

        # Define joint positions (radians)
        joint_cmd.position = [
            left_shoulder,
            self.elbow_angle,
            right_shoulder,
            self.elbow_angle
        ]

        # Note: velocity and effort fields are optional
        # For position control, we only need position

        # Publish joint commands
        self.joint_pub.publish(joint_cmd)

        # Log command for debugging
        self.get_logger().info(
            f'Command: {detection:5s} → '
            f'L_shoulder={math.degrees(left_shoulder):5.1f}°, '
            f'R_shoulder={math.degrees(right_shoulder):5.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    agent = ControlAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Shutting down Control Agent')
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
