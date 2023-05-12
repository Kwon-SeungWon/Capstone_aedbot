#! /usr/bin/env python3

import time  # Time library

from geometry_msgs.msg import PoseStamped  # Pose with ref frame and timestamp
from rclpy.duration import Duration  # Handles time for ROS 2
import rclpy  # Python client library for ROS 2
from rclpy.node import Node

from aedbot_interfaces.msg import Bridge, FallDetectionToNav2
from robot_navigator import BasicNavigator, NavigationResult  # Helper module

"""
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
"""


class GotoGoal(Node):
    def __init__(self):
        super().__init__("Gotogoal_Subscriber")
        self.subscription = self.create_subscription(
            FallDetectionToNav2, "dest_val", self.dest_val_callback, 10
        )
        self.subscription

    def dest_val_callback(self, destination):
        self.get_logger().info(
            "Incoming Destination is \nx: %f\n y: %f\n z: %f\n w: %f\n"
            % (
                destination.dest_x,
                destination.dest_y,
                destination.dest_z,
                destination.dest_w,
            )
        )

        self.destination_x = destination.dest_x
        self.destination_y = destination.dest_y
        self.destination_z = destination.dest_z
        self.destination_w = destination.dest_w

        self.go_to_destination()

    def go_to_destination(self):
        # Launch the ROS 2 Navigation Stack
        navigator = BasicNavigator()

        # Set the robot's initial pose if necessary
        initial_poses = []
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.initial_pose = initial_poses.append(initial_pose)
        navigator.setInitialPose(initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        navigator.lifecycleStartup()

        # Wait for navigation to fully activate. Use this line if autostart is set to true.
        # navigator.waitUntilNav2Active()

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        #####################################################

        # 주기적으로 local, global costmap clear (5s 마다)
        navigator.clear_periodically_costmap()

        navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()
        ######################################################

        goal_poses = []
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.destination_x
        goal_pose.pose.position.y = self.destination_y
        goal_pose.pose.orientation.z = self.destination_z
        goal_pose.pose.orientation.w = self.destination_w
        goal_poses.append(goal_pose)

        self.get_logger().info("GOGOGOGOGOGOGOOGGOOGGO")

        # LET'S GO
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)

        i = 0

        while not navigator.isNavComplete():
            #################################################
            #                                               #
            # Implement some code here for your application!#
            #                                               #
            #################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    "Executing current waypoint: "
                    + str(feedback.current_waypoint + 1)
                    + "/"
                    + str(len(goal_poses))
                )
                now = navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=100000000.0):
                    navigator.cancelNav()

                # Some follow waypoints request change to demo preemption
                if now - nav_start > Duration(seconds=500000.0):
                    goal_pose_alt = PoseStamped()
                    goal_pose_alt.header.frame_id = "map"
                    goal_pose_alt.header.stamp = now.to_msg()
                    goal_pose_alt.pose.position.x = 0.0
                    goal_pose_alt.pose.position.y = 0.0
                    goal_pose_alt.pose.position.z = 0.0
                    goal_pose_alt.pose.orientation.x = 0.0
                    goal_pose_alt.pose.orientation.y = 0.0
                    goal_pose_alt.pose.orientation.z = 0.0
                    goal_pose_alt.pose.orientation.w = 1.0
                    goal_poses = [goal_pose_alt]
                    nav_start = now
                    navigator.followWaypoints(goal_poses)

        # Do something depending on the return code
        self.result = navigator.getResult()
        if self.result == NavigationResult.SUCCEEDED:
            print("Goal succeeded!")
        elif self.result == NavigationResult.CANCELED:
            print("Goal was canceled!")
        elif self.result == NavigationResult.FAILED:
            print("Goal failed!")
        else:
            print("Goal has an invalid return status!")

        navigator.lifecycleShutdown()

        exit(0)

    def back_to_station(self):
        # Launch the ROS 2 Navigation Stack
        navigator = BasicNavigator()
        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # navigator.lifecycleStartup()

        # Wait for navigation to fully activate. Use this line if autostart is set to true.
        navigator.waitUntilNav2Active()

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        #####################################################

        # 주기적으로 local, global costmap clear (5s 마다)
        navigator.clear_periodically_costmap()

        navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()
        ######################################################

        # LET'S GO
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(self.initial_pose)

        i = 0

        while not navigator.isNavComplete():
            #################################################
            #                                               #
            # Implement some code here for your application!#
            #                                               #
            #################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    "Executing current waypoint: "
                    + str(feedback.current_waypoint + 1)
                    + "/"
                    + str(len(goal_poses))
                )
                now = navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=100000000.0):
                    navigator.cancelNav()

                # Some follow waypoints request change to demo preemption
                if now - nav_start > Duration(seconds=500000.0):
                    goal_pose_alt = PoseStamped()
                    goal_pose_alt.header.frame_id = "map"
                    goal_pose_alt.header.stamp = now.to_msg()
                    goal_pose_alt.pose.position.x = 0.0
                    goal_pose_alt.pose.position.y = 0.0
                    goal_pose_alt.pose.position.z = 0.0
                    goal_pose_alt.pose.orientation.x = 0.0
                    goal_pose_alt.pose.orientation.y = 0.0
                    goal_pose_alt.pose.orientation.z = 0.0
                    goal_pose_alt.pose.orientation.w = 1.0
                    goal_poses = [goal_pose_alt]
                    nav_start = now
                    navigator.followWaypoints(self.initial_pose)

        # Do something depending on the return code
        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print("Goal succeeded!")
        elif result == NavigationResult.CANCELED:
            print("Goal was canceled!")
        elif result == NavigationResult.FAILED:
            print("Goal failed!")
        else:
            print("Goal has an invalid return status!")

        navigator.lifecycleShutdown()

        exit(0)


class Bridge_to_Web_CPR(Node):
    def __init__(self):
        super().__init__("Bridge_Node")

        self.subscription = self.create_subscription(
            Bridge, "situation_end", self.listener_callback, 10  # CHANGE
        )
        self.subscription

        if BasicNavigator.getResult == 1:
            self.pubslisher = self.create_publisher(Bridge, "arrive_dest", 10)
            timer_period = 1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0
        ## 추후 서브스크라이버 추가

    def timer_callback(self):
        msg = Bridge()
        msg.arrive_destination = True
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.arrive_destination)
        self.i += 1

        if self.i == 5:
            exit(0)

    def listener_callback(self, msg):
        if msg.complete_cpr and msg.complete_web == True:
            GotoGoal.back_to_station()


def main():
    # Start the ROS 2 Python Client Library
    rclpy.init()

    # bridge 노드 실행
    bridge = Bridge_to_Web_CPR()
    # Service 노드 실행
    go_to_goal = GotoGoal()

    rclpy.spin(go_to_goal)
    rclpy.spin(bridge)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
