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


def Arrive_sub(v):
    qos_profile = QoSProfile(depth=10)

    Node.create_subscription(
        Bridge,
        "arrive_dest",
        partial(sub_callback_done, v=v),
        qos_profile,
    )


def sub_callback_done(msg, v):
    """
    state가 True가 되면 get_face() 함수가 종료됨
    """
    print("get_sub")
    v.value = True


def get_face(self):
    # launch firefox in a subprocess
    p = subprocess.Popen(["firefox", URL, "--kiosk"])

    while True:
        print(self.state)
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
    v = Value("B", False)
    p = Process(target=Arrive_sub, args=v)
    p.start()

    rclpy.init()
    node = Sub()
    node.state = v.value

    # if args.debug:
    #     node.listener_callback_get_dest(msg=None)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
