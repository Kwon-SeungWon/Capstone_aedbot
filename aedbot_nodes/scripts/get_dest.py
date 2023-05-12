#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from aedbot_interfaces.srv import FallDetectionToNav2


def get_dest():
    x, y, z, w = 1.0, 2.0, 3.0, 4.0
    return x, y, z, w


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("get_dest")
        self.publisher_ = self.create_publisher(
            FallDetectionToNav2, "dest_val", 10
        )  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = FallDetectionToNav2()
        x, y, z, w = get_dest()

        msg.dest_x = x
        msg.dest_y = y
        msg.dest_z = z
        msg.dest_w = w

        self.publisher_.publish(msg)
        self.get_logger().info(f"I pub: {msg.x}, {msg.y}, {msg.z}, {msg.w}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
