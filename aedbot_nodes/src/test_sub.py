#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from aedbot_interfaces.srv import FallDetectionToNav2


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            FallDetectionToNav2,
            'dest_val',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.x}, {msg.y}, {msg.z}, {msg.w}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()