#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import playsound  # pip3 install playsound
import os

from aedbot_interfaces.msg import Bridge

# get 2x upper directory
UPPER_DIR_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SIREN_PATH = os.path.join(UPPER_DIR_PATH, "music", "siren.mp3")


class Sub(Node):
    def __init__(self):
        super().__init__("sub")
        qos_profile = QoSProfile(depth=10)

        self.subscription = self.create_subscription(
            Bridge, "arrive_dest", self.listener_callback_predict, qos_profile
        )

        self.nav_done = False

    def listener_callback_predict(self, msg):
        if msg.arrive_destination is True:
            self.nav_done = True

        # print("I heard: {}".format(msg.data))


def main():
    rclpy.init()
    node = Sub()

    while True:
        if node.nav_done:
            break

        rclpy.spin_once(node, timeout_sec=0.1)
        playsound.playsound(SIREN_PATH, False)


if __name__ == "__main__":
    main()
