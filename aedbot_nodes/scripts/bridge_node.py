#! /usr/bin/env python3
import time  # Time library

from geometry_msgs.msg import PoseStamped  # Pose with ref frame and timestamp
from rclpy.duration import Duration  # Handles time for ROS 2
import rclpy  # Python client library for ROS 2
from rclpy.node import Node
from aedbot_interfaces.msg import Bridge, FallDetectionToNav2
from std_msgs.msg import Int32


class Bridge_to_Web_CPR(Node):
    def __init__(self):
        super().__init__("Bridge_Node")

        # nav2에서 bridge로 도착신호 받을 때
        # Topic은 arrive_dest_bridge
        self.subscription = self.create_subscription(
            Bridge, "arrive_dest_bridge", self.arrive_callback, 10
        )
        self.subscription

        # bridge에서 CPR, WEB으로 도착신호 보낼 때
        # Topic은 arriv_dest
        # msg는 Bridge의 arrive_destination이 True
        self.publisher_to_CPR = self.create_publisher(Bridge, "arrive_dest", 10)

        # CPR, WEB 통신이 끝났을 때
        # Topic은 situation_end
        # msg는 Bridge의 complete_cpr, complete_web
        self.subscription = self.create_subscription(
            Bridge, "situation_end", self.end_callback, 10  # CHANGE
        )
        self.subscription

        # CPR, WEB 통신이 끝났을 때 bridge에서 nav2로 복귀신호
        # Topic은 arrive_dest_bridge
        # msg는 Bridge의 bridge_to_nav2
        self.publisher_to_nav2 = self.create_publisher(Bridge, "arrive_dest_bridge", 10)

        self.cpr_state: bool = False
        self.web_state: bool = False

    def arrive_callback(self, msg: Bridge):
        # msg = Bridge()

        if msg.nav2_to_bridge == True:
            print("arrive callback")
            # msg.nav2_to_bridge = True
            msg.arrive_destination = True
            self.publisher_to_CPR.publish(msg)
            print(msg)

        else:
            msg.arrive_destination = False
            self.publisher_to_CPR.publish(msg)
            print(msg)

        self.get_logger().info('Publishing: "%d"' % msg.arrive_destination)

    def end_callback(self, msg: Bridge):
        # msg = Bridge()
        print("end callback")

        if msg.complete_cpr == 1:
            self.cpr_state = True

        if msg.complete_web is True:
            self.web_state = True

        if self.cpr_state is True and self.web_state is True:
            pub_msg = Bridge()
            pub_msg.bridge_to_nav2 = True
            self.publisher_to_nav2.publish(pub_msg)
            print("last publish success!!")
            return None


def main(args=None):
    # Start the ROS 2 Python Client Library
    rclpy.init(args=args)

    bridge = Bridge_to_Web_CPR()

    rclpy.spin(bridge)
    bridge.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
