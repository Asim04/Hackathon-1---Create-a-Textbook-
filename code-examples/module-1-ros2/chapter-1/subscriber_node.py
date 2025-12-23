#!/usr/bin/env python3
"""
Simple Subscriber Node
Purpose: Subscribes to /hello_topic and prints received messages
Prerequisites: ROS 2 Humble, Python 3.10+
Usage: python3 subscriber_node.py (run publisher_node.py in another terminal)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """
    A node that subscribes to /hello_topic and processes received messages.

    Key Concepts:
    - create_subscription(): Sets up a subscriber for a specific topic
    - Callback function: Automatically called when a message arrives
    - Asynchronous processing: Callbacks execute whenever messages arrive
    """

    def __init__(self):
        super().__init__('subscriber_node')

        # Create a subscription to /hello_topic
        # Parameters: message_type, topic_name, callback_function, queue_size
        # listener_callback is called automatically when a message arrives
        self.subscription = self.create_subscription(
            String,
            '/hello_topic',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning (subscription must be kept alive)
        self.subscription

        self.get_logger().info('Subscriber node started. Listening to /hello_topic.')

    def listener_callback(self, msg):
        """
        Called automatically whenever a message is received on /hello_topic.

        Args:
            msg (std_msgs.msg.String): The received message
        """
        # Access message data and log it
        self.get_logger().info(f'Received: "{msg.data}"')

        # In a real robot system, here you would process the data
        # For example: update robot state, trigger actuator commands, etc.


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    # spin() blocks here, processing callbacks as messages arrive
    # The node stays alive, waiting for messages on /hello_topic
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
