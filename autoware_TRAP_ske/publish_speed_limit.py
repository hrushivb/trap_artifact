#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tier4_planning_msgs.msg import VelocityLimit  # Ensure this message type is available in your workspace


class VelocityLimitEnforcer(Node):
    def __init__(self):
        super().__init__('velocity_limit_enforcer')

        # Define QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Publisher to enforce velocity limit
        self.publisher_ = self.create_publisher(
            VelocityLimit,
            '/planning/scenario_planning/max_velocity_default',
            qos_profile
        )

        # Subscriber to monitor current velocity limit
        self.subscriber_ = self.create_subscription(
            VelocityLimit,
            '/planning/scenario_planning/current_max_velocity',
            self.check_and_enforce_velocity_limit,
            qos_profile
        )

        # Desired velocity limit (adjust as needed)
        self.desired_velocity_limit = 10.0  # Example: 10 m/s

    def check_and_enforce_velocity_limit(self, msg):
        """Callback to check and enforce the desired velocity limit."""
        current_limit = msg.max_velocity

        if abs(current_limit - self.desired_velocity_limit) > 0.01:
            self.get_logger().info(f"Current velocity limit ({current_limit} m/s) does not match desired limit ({self.desired_velocity_limit} m/s). Enforcing...")
            self.publish_velocity_limit()
        else:
            self.get_logger().info(f"Velocity limit is correct: {current_limit} m/s")

    def publish_velocity_limit(self):
        """Publish the desired velocity limit."""
        msg = VelocityLimit()
        msg.stamp = self.get_clock().now().to_msg()
        msg.max_velocity = self.desired_velocity_limit

        # Log and publish the message
        self.get_logger().info(f"Publishing desired velocity limit: {self.desired_velocity_limit} m/s")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    node = VelocityLimitEnforcer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
