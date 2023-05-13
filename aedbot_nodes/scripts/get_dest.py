#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json

from aedbot_interfaces.msg import FallDetectionToNav2


URL = "http://130.162.152.119"


def get_dest():
    r = requests.get(url=URL + "/get_dest")
    data = r.json()

    x = data["x"]
    y = data["y"]
    z = data["z"]
    w = data["w"]

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
        destination = FallDetectionToNav2()
        x, y, z, w = get_dest()

        destination.dest_x = x
        destination.dest_y = y
        destination.dest_z = z
        destination.dest_w = w

        self.publisher_.publish(destination)
        self.get_logger().info(
            f"I pub: {destination.dest_x}, {destination.dest_y}, {destination.dest_z}, {destination.dest_w}"
        )
        self.i += 1

        if self.i == 2:
            exit(0)


def main():
    rclpy.init()

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
