import rclpy
from rclpy.node import Node
import requests
import os
import time
import subprocess
from rclpy.qos import QoSProfile
import argparse
from functools import partial
import datetime
from pytz import timezone

from aedbot_interfaces.msg import FallDetectionToNav2, Bridge


URL = "http://130.162.152.119"
TIME_FORMAT = "%Y-%m-%d_%H:%M:%S"
KST = timezone("Asia/Seoul")

parser = argparse.ArgumentParser()
parser.add_argument("--debug", action="store_true", help="debug mode")
args = parser.parse_args()


def get_time_diff(URL):
    r = requests.get(url=URL)
    data = r.json()
    server_time = data["time"]

    current_time = datetime.datetime.now(KST).strftime(TIME_FORMAT)
    time_diff = datetime.datetime.strptime(
        current_time, TIME_FORMAT
    ) - datetime.datetime.strptime(server_time, TIME_FORMAT)

    if abs(time_diff.total_seconds()) < 5:
        return True

    return False


def get_face():
    # launch firefox in a subprocess
    p = subprocess.Popen(["firefox", URL + "/HRI", "--kiosk"])

    while True:
        if get_time_diff(URL + "/get_arrive"):
            break

        time.sleep(0.01)

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
        get_face()


def main():
    rclpy.init()
    node = Sub()

    if args.debug:
        node.listener_callback_get_dest(msg=None)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
