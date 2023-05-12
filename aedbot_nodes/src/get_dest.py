#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from contextlib import suppress

from aedbot_interfaces.srv import FallDetectionToNav2


def get_dest():
    x, y, z, w = 1.0, 2.0, 3.0, 4.0
    return x, y, z, w


def main(args=None):
    prev_time = time.time()
    rclpy.init()

    node = rclpy.create_node("get_dest_node")
    publisher = node.create_publisher(FallDetectionToNav2, "dest_val", 10)

    x, y, z, w = get_dest()
    with suppress(KeyboardInterrupt):
        while True:
            now_time = time.time()
            if abs(prev_time - now_time) > 0.1:
                msg = FallDetectionToNav2()
                publisher.publish(msg)
            prev_time = now_time

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
