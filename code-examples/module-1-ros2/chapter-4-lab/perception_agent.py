#!/usr/bin/env python3
"""
Lab Perception Agent
Purpose: Process left/right distance sensors and publish obstacle detection messages
Prerequisites: ROS 2 Humble, Python 3.10+
Usage: python3 perception_agent.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class PerceptionAgent(Node):
    """
    Perception agent for humanoid robot obstacle detection.

    Subscribes to:
        /left_distance (Float32): Distance from left arm sensor (meters)
        /right_distance (Float32): Distance from right arm sensor (meters)

    Publishes to:
        /obstacle_detected (String): Detection result ("left", "right", "both", "none")

    Key Concepts:
    - Multi-sensor fusion (combining left + right sensors)
    - Threshold-based detection (simple but effective)
    - Periodic processing using timer callback (decouples sensor rate from processing rate)
    """

    def __init__(self):
        super().__init__('perception_agent')

        # Subscribers for left/right distance sensors
        self.left_sub = self.create_subscription(
            Float32,
            '/left_distance',
            self.left_callback,
            10
        )
        self.right_sub = self.create_subscription(
            Float32,
            '/right_distance',
            self.right_callback,
            10
        )

        # Publisher for obstacle detection
        self.detection_pub = self.create_publisher(
            String,
            '/obstacle_detected',
            10
        )

        # Sensor state (latest readings)
        self.left_distance = None
        self.right_distance = None

        # Detection parameters
        self.detection_threshold = 0.5  # meters - obstacles closer than this trigger detection

        # Timer for periodic detection processing (10 Hz)
        # This decouples sensor message rate from detection logic rate
        self.timer = self.create_timer(0.1, self.process_detection)

        self.get_logger().info('Perception Agent started')
        self.get_logger().info(f'Detection threshold: {self.detection_threshold}m')
        self.get_logger().info('Waiting for sensor data on /left_distance and /right_distance...')

    def left_callback(self, msg):
        """
        Update left sensor reading.

        Args:
            msg (std_msgs.msg.Float32): Distance measurement from left sensor
        """
        self.left_distance = msg.data

    def right_callback(self, msg):
        """
        Update right sensor reading.

        Args:
            msg (std_msgs.msg.Float32): Distance measurement from right sensor
        """
        self.right_distance = msg.data

    def process_detection(self):
        """
        Process sensor data and publish detection message.

        Called at 10 Hz by timer. Uses latest sensor readings to determine
        if obstacles are present on left side, right side, both, or neither.

        Detection Logic:
        - left_distance < threshold → left obstacle
        - right_distance < threshold → right obstacle
        - both < threshold → both obstacles
        - neither < threshold → no obstacles
        """
        # Wait until we have data from both sensors
        if self.left_distance is None or self.right_distance is None:
            return

        # Apply threshold to detect obstacles
        left_obstacle = self.left_distance < self.detection_threshold
        right_obstacle = self.right_distance < self.detection_threshold

        # Determine detection result
        if left_obstacle and right_obstacle:
            detection = "both"
        elif left_obstacle:
            detection = "left"
        elif right_obstacle:
            detection = "right"
        else:
            detection = "none"

        # Publish detection message
        msg = String()
        msg.data = detection
        self.detection_pub.publish(msg)

        # Log detection (only when obstacles are detected to reduce spam)
        if detection != "none":
            self.get_logger().info(
                f'Obstacle detected: {detection} '
                f'(Left: {self.left_distance:.2f}m, Right: {self.right_distance:.2f}m)'
            )


def main(args=None):
    rclpy.init(args=args)
    agent = PerceptionAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Shutting down Perception Agent')
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
