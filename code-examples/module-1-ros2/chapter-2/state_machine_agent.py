#!/usr/bin/env python3
"""
State Machine Navigation Agent
Purpose: Demonstrate stateful agent with IDLE, MOVING, STOPPED states
Prerequisites: ROS 2 Humble, Python 3.10+
Usage: python3 state_machine_agent.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from enum import Enum


class RobotState(Enum):
    """States for robot navigation state machine"""
    IDLE = 0
    MOVING = 1
    STOPPED = 2


class StateMachineAgent(Node):
    """
    Agent with state machine for navigation control.

    States:
        IDLE: Waiting for start command (distance must exceed safe threshold)
        MOVING: Moving forward while distance > safe_distance
        STOPPED: Stopped due to obstacle, waiting for clearance

    Transitions:
        IDLE → MOVING: distance > safe_distance
        MOVING → STOPPED: distance <= safe_distance
        STOPPED → MOVING: distance > resume_distance (hysteresis)

    Key Concepts:
    - State machine pattern for complex behaviors
    - Hysteresis to prevent oscillation between states
    - Separation of state transition logic from action execution
    """

    def __init__(self):
        super().__init__('state_machine_agent')

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Float32,
            '/distance_sensor',
            self.distance_callback,
            10
        )
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # State machine configuration
        self.state = RobotState.IDLE
        self.safe_distance = 1.0      # Stop threshold (meters)
        self.resume_distance = 1.5    # Resume threshold (meters) - prevents oscillation

        self.get_logger().info(f'State Machine Agent started in state: {self.state.name}')
        self.get_logger().info(f'Safe distance: {self.safe_distance}m, Resume distance: {self.resume_distance}m')

    def distance_callback(self, msg):
        """
        Process distance sensor data and update state machine.

        Args:
            msg (std_msgs.msg.Float32): Distance measurement
        """
        distance = msg.data

        # Update state based on current state and distance
        new_state = self.update_state(distance)

        # Log state transitions
        if new_state != self.state:
            self.get_logger().info(
                f'State transition: {self.state.name} → {new_state.name} '
                f'(distance: {distance:.2f}m)'
            )
            self.state = new_state

        # Execute action for current state
        velocity = self.execute_state_action()
        self.velocity_publisher.publish(velocity)

    def update_state(self, distance):
        """
        State transition logic based on current state and distance.

        Implements hysteresis: resume_distance > safe_distance prevents
        rapid switching between MOVING and STOPPED states when distance
        hovers near the threshold.

        Args:
            distance (float): Current distance to obstacle (meters)

        Returns:
            RobotState: New state (may be same as current state if no transition)
        """
        if self.state == RobotState.IDLE:
            # Start moving when path is clear
            if distance > self.safe_distance:
                return RobotState.MOVING

        elif self.state == RobotState.MOVING:
            # Stop when obstacle detected
            if distance <= self.safe_distance:
                return RobotState.STOPPED

        elif self.state == RobotState.STOPPED:
            # Resume only when obstacle is sufficiently far (hysteresis)
            if distance > self.resume_distance:
                return RobotState.MOVING

        # No transition - remain in current state
        return self.state

    def execute_state_action(self):
        """
        Execute action associated with current state.

        Returns:
            geometry_msgs/Twist: Velocity command for current state
        """
        velocity = Twist()

        if self.state == RobotState.IDLE:
            velocity.linear.x = 0.0
            self.get_logger().debug('State: IDLE - Waiting to start')

        elif self.state == RobotState.MOVING:
            velocity.linear.x = 0.5  # m/s
            self.get_logger().debug('State: MOVING - Forward at 0.5 m/s')

        elif self.state == RobotState.STOPPED:
            velocity.linear.x = 0.0
            self.get_logger().debug('State: STOPPED - Obstacle detected')

        velocity.angular.z = 0.0  # No rotation
        return velocity


def main(args=None):
    rclpy.init(args=args)
    agent = StateMachineAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Shutting down State Machine Agent')
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
