#!/usr/bin/env python3
"""
Simple Publisher Node for AMR Learning
Publishes a heartbeat status message to the topic '/robot_status' at 1 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        # 1. Initialize the node with the name 'simple_publisher'
        super().__init__('simple_publisher')

        # 2. Create a publisher:
        #    - Message type: String
        #    - Topic name: 'robot_status'
        #    - QoS history depth (queue size): 10
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)

        # 3. Create a timer running every 1.0 second (1 Hz frequency)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 4. State counter
        self.counter = 0
        self.get_logger().info('SimplePublisher node has started. Publishing to /robot_status...')

    def timer_callback(self):
        """Called automatically every 1.0 second by the timer."""
        msg = String()
        msg.data = f'AMR System Online - Ping #{self.counter}'
        
        # Publish message to the topic
        self.publisher_.publish(msg)

        # Log output to console
        self.get_logger().info(f'Published: "{msg.data}"')
        self.counter += 1


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Instantiate the node
    node = SimplePublisher()

    try:
        # Keep the node running to process callbacks (timers, subscriptions, etc.)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
