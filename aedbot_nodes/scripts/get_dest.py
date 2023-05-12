#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import json
import requests

from aedbot_interfaces.srv import FallDetectionToNav2

URL = "http://130.162.152.119"


def get_dest():
    # r = requests.get(url=URL + "/get_dest")
    # data = r.json()

    # x = data["x"]
    # y = data["y"]
    # z = data["z"]
    # w = data["w"]
    x, y, z, w = 3.0, 4.0, 5.0, 6.0
    return x, y, z, w


def main(args=None):
    prev_time = time.time()
    rclpy.init()

    node = rclpy.create_node("get_dest_node")
    publisher = node.create_publisher(FallDetectionToNav2, "dest_val", 10)

    while True:
        now_time = time.time()
        if abs(prev_time - now_time) > 0.1:
            msg = FallDetectionToNav2()

            x, y, z, w = get_dest()

            msg.dest_x = x
            msg.dest_y = y
            msg.dest_z = z
            msg.dest_w = w

            publisher.publish(msg)

            rclpy.get_logger().info(
                f"I pub: {msg.dest_x}, {msg.dest_y}, {msg.dest_z}, {msg.dest_w}"
            )

        prev_time = now_time

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
