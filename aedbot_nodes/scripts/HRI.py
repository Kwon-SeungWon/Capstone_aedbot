import rclpy
from rclpy.node import Node
import requests
import os
import time
import subprocess
from rclpy.qos import QoSProfile
import argparse

from aedbot_interfaces.msg import FallDetectionToNav2, Bridge


URL = "http://130.162.152.119/HRI"

parser = argparse.ArgumentParser()
parser.add_argument("debug", type=bool, action="store_true", help="debug mode")
args = parser.parse_args()


def get_face(self):
    # launch firefox in a subprocess
    p = subprocess.Popen(["firefox", URL, "--kiosk"])

    while True:  # TODO: state가 while문 안에서 바뀌는지 확인
        if self.state:
            break

    # terminate the subprocess if it's still running

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

        self.subscription_done = self.create_subscription(
            Bridge,
            "arrive_dest",
            self.listener_callback_done,
            qos_profile,
        )

        self.state = False
        self.callback_count = False  # callback이 두번 호출되는 것을 방지하기 위함

    def listener_callback_get_dest(self, msg):
        if self.callback_count:
            """
            callback이 두번 호출되는 것을 방지하기 위함
            """
            return None

        self.callback_count = True
        get_face(self)

    def listener_callback_done(self, msg):
        """
        state가 True가 되면 get_face() 함수가 종료됨
        """
        self.state = True


def main(args=None):
    rclpy.init(args=args)
    node = Sub()

    if args.debug:
        node.state = True
        get_face(node)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
