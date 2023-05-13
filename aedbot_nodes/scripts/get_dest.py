#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json
import datetime
from pytz import timezone  # pip3 install pytz

from aedbot_interfaces.msg import FallDetectionToNav2


URL = "http://130.162.152.119"
TIME_FORMAT = "%Y-%m-%d_%H:%M:%S"
KST = timezone("Asia/Seoul")


def get_dest():
    r = requests.get(url=URL + "/get_dest")
    data = r.json()

    x = data["x"]
    y = data["y"]
    z = data["z"]
    w = data["w"]
    time = data["time"]

    return x, y, z, w, time


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("get_dest")
        self.publisher_ = self.create_publisher(
            FallDetectionToNav2, "dest_val", 10
        )  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.state = False

    def timer_callback(self):
        if self.state:
            return None

        destination = FallDetectionToNav2()
        x, y, z, w, time = get_dest()

        current_time = datetime.datetime.now(KST).strftime(TIME_FORMAT)
        time_diff = datetime.datetime.strptime(
            current_time, TIME_FORMAT
        ) - datetime.datetime.strptime(time, TIME_FORMAT)

        time_diff = abs(time_diff.total_seconds())  # float
        self.get_logger().info(f"time_diff: {time_diff}")

        if time_diff <= 10:
            destination.dest_x = x
            destination.dest_y = y
            destination.dest_z = z
            destination.dest_w = w

            for _ in range(5):
                self.publisher_.publish(destination)
                self.get_logger().info(
                    f"I pub: {destination.dest_x}, {destination.dest_y}, {destination.dest_z}, {destination.dest_w}"
                )

            self.state = True


def main():
    rclpy.init()

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
