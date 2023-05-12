#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from contextlib import suppress
from rclpy.qos import QoSProfile

from aedbot_interfaces.srv import FallDetectionToNav2


class Sub(Node):
    def __init__(self):
        super().__init__("sub")
        qos_profile = QoSProfile(depth=10)

        self.subscription = self.create_subscription(
            FallDetectionToNav2, "get_dest", self.listener_callback_predict, qos_profile
        )

    def listener_callback_predict(self, msg):
        # append and delete
        print("I heard: {}".format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = Sub()

    while True:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
