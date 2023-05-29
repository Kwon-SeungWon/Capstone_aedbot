#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
from pytz import timezone  # pip3 install pytz
from std_msgs.msg import Float64
from aedbot_interfaces.msg import FallDetectionToNav2

TIME_FORMAT = "%Y-%m-%d_%H:%M:%S"
KST = timezone("Asia/Seoul")


def get_dest():
    x = 0.0
    y = -0.5
    z = -0.7085
    w = 0.705

    return x, y, z, w


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("get_dest")
        self.publisher_ = self.create_publisher(
            FallDetectionToNav2, "dest_val", 10
        )  # CHANGE
        self.publisher_arduino = self.create_publisher(
            Float64, "dest_val_arduino", 10
        )  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.state = False

    def timer_callback(self):
        if self.state:
            return None

        destination = FallDetectionToNav2()
        msg = Float64()
        x, y, z, w = get_dest()

        destination.dest_x = x
        destination.dest_y = y
        destination.dest_z = z
        destination.dest_w = w

        for _ in range(1):
            self.publisher_.publish(destination)
            self.publisher_arduino.publish(msg)
            self.get_logger().info(
                f"I pub: {destination.dest_x}, {destination.dest_y}, {destination.dest_z}, {destination.dest_w}"
            )
        exit(0)

        self.state = True


def main():
    rclpy.init()

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
