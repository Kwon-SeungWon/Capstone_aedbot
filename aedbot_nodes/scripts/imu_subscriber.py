#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from aedbot_interfaces.msg import Bridge

import time
from playsound import playsound


class ImuSubscriberNode(Node):
    def __init__(self):
        super().__init__("imu_subscriber")
        self.imu_subscriber2_ = self.create_subscription(
            Bridge, "arrive_dest", self.receiver_callback, 10
        )

        # self.imu_subscriber_ = self.create_subscription(
        #     Imu, "/imu/data", self.imu_callback, 10)

        self.publisher_ = self.create_publisher(
            Int32, "micro_ros_arduino_subscriber", 10
        )
        self.i = 0
        self.j = 0
        self.k = 0
        self.count_msg = 0
        self.start_time = time.monotonic()
        self.elapsed_time = 0

    def receiver_callback(self, msg: Bridge):
        self.recieve_result = msg.arrive_destination

        if self.recieve_result == True:
            print("Receive Complete")
            print(msg.arrive_destination)

            self.imu_subscriber_ = self.create_subscription(
                Imu, "cpr_imu/data", self.imu_callback, 10)

    def imu_callback(self, msg: Imu):
        # self.k= Bool.data
        # self.get_logger().info('self.k:%s' %type(self.k))
        print("imu_callback Complete")
        #playsound("/root/catkin_ws/src/aedbot/aedbot_nodes/music/120bpm.mp3", True)

        count_msg = Int32()
        self.get_logger().info("count:%d" % count_msg.data)

        if msg.linear_acceleration.z > 13:
            self.i = 1
            count_msg.data = 0

        if msg.linear_acceleration.z < 1:
            if self.i == 1:
                count_msg.data = 3
                self.get_logger().info("count:%d" % count_msg.data)
                self.publisher_.publish(count_msg)
                self.i = 0

        if 1 <= msg.linear_acceleration.z < 5:
            if self.i == 1:
                count_msg.data = 2
                self.get_logger().info("count:%d" % count_msg.data)
                self.publisher_.publish(count_msg)
                self.i = 0

        if 5 <= msg.linear_acceleration.z < 6:
            if self.i == 1:
                count_msg.data = 1
                self.get_logger().info("count:%d" % count_msg.data)
                self.publisher_.publish(count_msg)
                self.i = 0


def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
