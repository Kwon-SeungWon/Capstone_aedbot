#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import playsound  # pip3 install playsound
import os

from aedbot_interfaces.msg import Bridge, FallDetectionToNav2

# get 2x upper directory
UPPER_DIR_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SIREN_PATH = os.path.join(UPPER_DIR_PATH, "music", "siren.mp3")


class Sub(Node):
    def __init__(self):
        super().__init__("sub")
        qos_profile = QoSProfile(depth=10)

        self.arrive_subscription = self.create_subscription(
            Bridge, "arrive_dest", self.listener_callback_predict, qos_profile
        )

        self.dest_subscription = self.create_subscription(
            FallDetectionToNav2, "dest_val", self.listener_callback_dest, qos_profile
        )

        self.nav_done = False
        self.nav_start = False

    def listener_callback_predict(self, msg):
        msg = Bridge()
        if msg.arrive_destination is True:
            self.nav_done = True

        return None

    def listener_callback_dest(self, msg):
        self.nav_start = True
        return None

        # print("I heard: {}".format(msg.data))


def main():
    os.system("sudo pulseaudio -k")
    os.system("pactl -- set-sink-volume 0 50%") #200%
    os.system("pulseaudio --start")
    rclpy.init()
    node = Sub()

    #if node.nav_done:
    #    break

    rclpy.spin_once(node, timeout_sec=0.1)

    playsound.playsound(SIREN_PATH, True)


if __name__ == "__main__":
    main()
