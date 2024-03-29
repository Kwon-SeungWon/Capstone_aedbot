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
        # Topic은 arrive_dest
        # msg는 Bridge의 arrive_destination이 True
        self.publisher_to_CPR = self.create_publisher(Bridge, "arrive_dest", 10)

        # CPR, WEB 통신이 끝났을 때
        # Topic은 situation_end
        # msg는 Bridge의 complete_cpr, complete_web
        self.subscription = self.create_subscription(
            Int32, "situation_end", self.cpr_end_callback, 10  # CHANGE
        )
        self.subscription

        self.subscription = self.create_subscription(
            Bridge, "web_end", self.web_end_callback, 10  # CHANGE
        )
        self.subscription

        self.cpr_state: bool = False
        self.web_state: bool = False

        # CPR, WEB 통신이 끝났을 때 bridge에서 nav2로 복귀신호
        # Topic은 arrive_dest_bridge
        # msg는 Bridge의 bridge_to_nav2
        self.publisher_to_nav2 = self.create_publisher(Bridge, "arrive_dest_bridge", 10)

    # int32 bridge_to_cpr
    def arrive_callback(self, msg: Bridge):
        # msg = Bridge()
        self.nav2_to_bridge = msg.nav2_to_bridge
        self.bridge_to_cpr = self.cpr_state

        if self.bridge_to_cpr:
            self.nav2_to_bridge = False

        # if self.nav2_to_bridge == True:

        if self.nav2_to_bridge == True:
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

    def cpr_end_callback(self, msg: Int32):
        print("cpr_end callback")
        # self.bridge_to_cpr = msg.data
        arrive_msg = Bridge()
        arrive_msg.arrive_destination = False
        arrive_msg.bridge_to_cpr = True
        arrive_msg.bridge_to_nav2 = True
        arrive_msg.complete_web = False
        arrive_msg.nav2_to_bridge = False

        self.publisher_to_CPR.publish(arrive_msg)

            
        self.publish_to_station()

    def web_end_callback(self, msg: Bridge):
        # msg = Bridge()
        print("web_end callback")

        if msg.complete_web is True:
            self.web_state = True
            self.publish_to_station()

    def publish_to_station(self):
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
