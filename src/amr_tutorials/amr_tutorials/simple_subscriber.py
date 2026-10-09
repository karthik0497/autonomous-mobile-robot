#!/usr/bin/env python3
"""
Simple Subscriber Node for AMR Learning
Subscribes to the topic '/robot_status' and logs any received messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        # 1. Initialize the node with the name 'simple_subscriber'
        super().__init__('simple_subscriber')

        # 2. Create a subscriber:
        #    - Message type: String
        #    - Topic name: 'robot_status'
        #    - Callback function: self.listener_callback
        #    - QoS history depth (queue size): 10
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10
        )
        # Prevent unused variable warning
        self.subscription

        self.get_logger().info('SimpleSubscriber node has started. Listening on /robot_status...')

    def listener_callback(self, msg: String):
        """Called automatically whenever a new message arrives on /robot_status."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Instantiate the node
    node = SimpleSubscriber()

    try:
        # Keep the node running to process incoming messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
