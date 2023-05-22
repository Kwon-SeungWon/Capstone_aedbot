import rclpy
from rclpy.node import Node
import requests
import os
import time
import subprocess
from rclpy.qos import QoSProfile
import argparse
from multiprocessing import Process, Value
from functools import partial

from aedbot_interfaces.msg import FallDetectionToNav2, Bridge


URL = "http://130.162.152.119/HRI"

parser = argparse.ArgumentParser()
parser.add_argument("--debug", action="store_true", help="debug mode")
args = parser.parse_args()


def run(state):
    def arrive_sub(sub_node):
        sub = sub_node.create_subscription(
            Bridge,
            "arrive_dest",
            sub_callback_done,
            10,
        )

    def sub_callback_done(msg):
        """
        state가 True가 되면 get_face() 함수가 종료됨
        """
        state.value = True

    rclpy.init()
    sub_node = Node("listener_node")
    arrive_sub(sub_node)
    rclpy.spin(sub_node)
    sub_node.destroy_node()
    rclpy.shutdown()


def get_face(self):
    # launch firefox in a subprocess
    p = subprocess.Popen(["firefox", URL, "--kiosk"])

    while True:
        if self.state.value:
            break

    p.terminate()
    return None


class Sub(Node):
    def __init__(self):
        super().__init__("sub")
        qos_profile = QoSProfile(depth=10)

        self.subscription_start = self.create_subscription(
            FallDetectionToNav2,
            "dest_val",
            self.listener_callback_get_dest,
            qos_profile,
        )

        self.callback_count = False  # callback이 두번 호출되는 것을 방지하기 위함

    def listener_callback_get_dest(self, msg):
        if self.callback_count:
            """
            callback이 두번 호출되는 것을 방지하기 위함
            """
            return None

        self.callback_count = True
        get_face(self)


def main():
    state = Value("B", False)

    rclpy.init()
    node = Sub()
    node.state = state

    p = Process(target=run, args=(state,))
    p.start()

    if args.debug:
        node.listener_callback_get_dest(msg=None)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
