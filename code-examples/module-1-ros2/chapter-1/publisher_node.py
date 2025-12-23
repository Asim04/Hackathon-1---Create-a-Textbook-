#!/usr/bin/env python3
"""
Simple Publisher Node
Purpose: Publishes "Hello ROS 2" messages every second to demonstrate topic publishing
Prerequisites: ROS 2 Humble, Python 3.10+
Usage: python3 publisher_node.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    """
    A node that publishes string messages to /hello_topic every second.

    Key Concepts:
    - create_publisher(): Sets up a publisher for a specific topic and message type
    - create_timer(): Schedules a callback function to run at regular intervals
    - publish(): Sends a message to all subscribers on the topic
    """

    def __init__(self):
        super().__init__('publisher_node')

        # Create a publisher for the /hello_topic
        # Parameters: message_type, topic_name, queue_size
        # queue_size=10 means buffer up to 10 messages if publishing faster than network can send
        self.publisher_ = self.create_publisher(String, '/hello_topic', 10)

        # Create a timer that calls timer_callback() every 1.0 second
        # This is the ROS 2 way to schedule periodic tasks
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Counter to track how many messages we've sent
        self.counter = 0

        self.get_logger().info('Publisher node started. Publishing to /hello_topic every 1 second.')

    def timer_callback(self):
        """
        Called every 1 second by the timer.
        Creates and publishes a message to /hello_topic.
        """
        # Create a message instance
        msg = String()
        msg.data = f'Hello ROS 2! Message #{self.counter}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log what we published (visible with ros2 topic echo or in console)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter for next message
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
