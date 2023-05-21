#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import requests
import os
import time
import subprocess
from rclpy.qos import QoSProfile
import datetime
from pytz import timezone  # pip3 install pytz

from aedbot_interfaces.msg import Bridge


TIME_FORMAT = "%Y-%m-%d_%H:%M:%S"
KST = timezone("Asia/Seoul")

FACETIME_URL = "http://xxx.xxx.xxx.xxx/"  # TODO: 추후 수정
QUIT_URL = "http://130.162.152.119/get_quit/"


def check_quit(seconds: float) -> bool:
    """
    서버에 접속해, 마지막으로 영상통화를 종료한 시간을 가져온다.
    마지막 종료한 시간과 현재 시간의 차가 (seconds)이면,
    종료한 것으로 판단. True 반환.
    """
    r = requests.get(url=QUIT_URL)
    data = r.json()

    quit_time = data["time"]
    current_time = datetime.datetime.now(KST).strftime(TIME_FORMAT)
    time_diff = datetime.datetime.strptime(
        current_time, TIME_FORMAT
    ) - datetime.datetime.strptime(quit_time, TIME_FORMAT)

    time_diff = abs(time_diff.total_seconds())  # float

    if time_diff <= seconds:
        return True

    return False


def connect_facetime():
    # launch firefox in a subprocess
    p = subprocess.Popen(["firefox", FACETIME_URL, "--kiosk"])

    while True:
        if check_quit(5):
            break

        time.sleep(0.1)

    # TODO: 영상통화 끝났다는 것 pub
    p.terminate()
    return None


class Sub(Node):
    def __init__(self):
        super().__init__("sub")
        qos_profile = QoSProfile(depth=10)

        self.subscription_done = self.create_subscription(
            Bridge,
            "arrive_dest",
            self.listener_callback_done,
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
        connect_facetime()


def main(args=None):
    rclpy.init(args=args)
    node = Sub()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()