import rclpy
from rclpy.node import Node
import requests
import time
import subprocess
from rclpy.qos import QoSProfile
import datetime
from pytz import timezone

from aedbot_interfaces.msg import FallDetectionToNav2


URL = "http://130.162.152.119"
TIME_FORMAT = "%Y-%m-%d_%H:%M:%S"
KST = timezone("Asia/Seoul")


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
        get_face()


def main():
    rclpy.init()
    node = Sub()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
