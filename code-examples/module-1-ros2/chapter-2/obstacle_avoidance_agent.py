#!/usr/bin/env python3
"""
Obstacle Avoidance Agent
Purpose: Subscribe to distance sensor, publish velocity commands based on distance
Prerequisites: ROS 2 Humble, Python 3.10+
Usage: python3 obstacle_avoidance_agent.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32       # Distance sensor message type
from geometry_msgs.msg import Twist    # Velocity command message type


class ObstacleAvoidanceAgent(Node):
    """
    Reactive agent that stops when obstacles are detected within 1 meter.

    Subscribes to:
        /distance_sensor (std_msgs/Float32): Distance to nearest obstacle in meters

    Publishes to:
        /cmd_vel (geometry_msgs/Twist): Velocity commands (linear.x, angular.z)

    Key Concepts:
    - Perceive-Reason-Act loop implementation
    - Callback-driven reactive control
    - Sensor-to-actuator pattern
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_agent')

        # PERCEIVE: Subscribe to distance sensor
        self.subscription = self.create_subscription(
            Float32,
            '/distance_sensor',
            self.distance_callback,
            10
        )

        # ACT: Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Agent state
        self.current_distance = None
        self.safe_distance = 1.0  # meters

        self.get_logger().info('Obstacle Avoidance Agent started')
        self.get_logger().info(f'Safe distance threshold: {self.safe_distance}m')

    def distance_callback(self, msg):
        """
        Called automatically when distance sensor data arrives.

        Implements the perceive-reason-act loop:
        1. PERCEIVE: Read distance from sensor
        2. REASON: Decide action based on distance
        3. ACT: Publish velocity command

        Args:
            msg (std_msgs.msg.Float32): Distance measurement from sensor
        """
        # PERCEIVE: Extract distance from message
        self.current_distance = msg.data

        # REASON: Apply decision logic
        velocity_cmd = self.decide_action(self.current_distance)

        # ACT: Publish velocity command
        self.velocity_publisher.publish(velocity_cmd)

        # Log decision for debugging
        self.get_logger().info(
            f'Distance: {self.current_distance:.2f}m → '
            f'Velocity: {velocity_cmd.linear.x:.2f}m/s'
        )

    def decide_action(self, distance):
        """
        Decision logic: Move forward if safe, stop if obstacle detected.

        This is a pure function - output depends only on input distance.
        Makes testing and debugging easier.

        Args:
            distance (float): Distance to nearest obstacle in meters

        Returns:
            geometry_msgs/Twist: Velocity command message
        """
        velocity = Twist()

        if distance is None:
            # No sensor data yet, remain stopped
            velocity.linear.x = 0.0
        elif distance > self.safe_distance:
            # Safe to move forward
            velocity.linear.x = 0.5  # m/s
        else:
            # Obstacle detected, stop
            velocity.linear.x = 0.0

        velocity.angular.z = 0.0  # No rotation for this simple example
        return velocity


def main(args=None):
    rclpy.init(args=args)
    agent = ObstacleAvoidanceAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Shutting down Obstacle Avoidance Agent')
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
