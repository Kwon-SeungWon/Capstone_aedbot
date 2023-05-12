#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import Bool
import time


class ImuSubscriberNode(Node):
    def __init__(self):
        super().__init__("imu_subscriber")
        self.imu_subscriber2_ = self.create_subscription(
            Bool, "/imu/data2", self.imu_callback2, 10
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

    def imu_callback2(self, msg: Bool):
        self.imu_subscriber_ = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, 10
        )

    def imu_callback(self, msg: Imu):
        self.k = Bool.data
        self.get_logger().info("self.k:%s" % type(self.k))
        self.get_logger().info("self.k:%s" % self.k)

        if self.k == True:
            # publish_msg = Int32()
            mean_count_state = Int32()
            # mean_count_state.data = self.k
            # publish_msg.data=self.i

            self.get_logger().info("count:%d" % self.count_msg)

            if msg.linear_acceleration.z > 13:
                # self.get_logger().info('accel is over 15: "%s' %msg.linear_acceleration.z)
                self.i = 1
                self.j = 0
                # self.get_logger().info('count1:%d' %publish_msg.data)
                # self.publisher_.publish(publish_msg)
            if msg.linear_acceleration.z < 5:
                # self.get_logger().info('accel is under 3: "%s' %msg.linear_acceleration.z)
                # self.get_logger().info('count2:%d' %publish_msg.data)
                if self.i == 1:
                    self.j += 1
                    self.count_msg += self.j
                    # self.publisher_.publish(count_msg)
                    self.get_logger().info("count:%d" % self.count_msg)
                    self.i = 0

            self.elapsed_time = time.monotonic() - self.start_time
            if 0 < self.elapsed_time < 5:
                self.get_logger().info("elapsed_time : %f" % self.elapsed_time)
                # self.get_logger().info('mean_count_state: %d'%mean_count_state.data)
            if self.elapsed_time > 5:
                if 8 <= self.count_msg <= 10:
                    self.get_logger().info(
                        "elapsed_Time:%f,self.mean_count:8~10" % self.elapsed_time
                    )
                    mean_count_state.data = 1
                    self.publisher_.publish(mean_count_state)
                    self.start_time = time.monotonic()
                    self.count_msg = 0
                elif self.count_msg > 10:
                    self.get_logger().info(
                        "elapsed_time: %f, self.mean_count:`10~" % self.elapsed_time
                    )
                    mean_count_state.data = 0
                    self.publisher_.publish(mean_count_state)
                    self.start_time = time.monotonic()
                    self.count_msg = 0
                elif self.count_msg < 8:
                    self.get_logger().info(
                        "elapsed_time: %f, self.mean_count:`~8" % self.elapsed_time
                    )
                    mean_count_state.data = 0
                    self.publisher_.publish(mean_count_state)
                    self.start_time = time.monotonic()
                    self.count_msg = 0


def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
