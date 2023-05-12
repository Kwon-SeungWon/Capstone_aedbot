#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# from aedbot_interfaces.srv import FallDetectionToNav2


def get_dest():
    x, y, z, w = 1.0, 2.0, 3.0, 4.0
    return x, y, z, w


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("get_dest")
        self.publisher_ = self.create_publisher(String, "dest_val", 10)  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        x, y, z, w = get_dest()

        msg.data = f"{x}, {y}, {z}, {w}"

        self.publisher_.publish(msg)
        self.get_logger().info(f"I pub: {msg.data}")


def main():
    rclpy.init()

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
