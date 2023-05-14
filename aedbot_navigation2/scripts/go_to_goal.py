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


class Go_to_Destination:
    def set_initial_pose(self):
        # Launch the ROS 2 Navigation Stack
        navigator = BasicNavigator()

        # Set the robot's initial pose if necessary
        # initial_poses = []
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = "map"
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.position.z = 0.0
        # initial_pose.pose.orientation.x = 0.0
        # initial_pose.pose.orientation.y = 0.0
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 1.0
        # self.initial_pose = initial_poses.append(initial_pose)
        # navigator.setInitialPose(initial_pose)

        # Position(-0.868823, -0.701858, 0), Orientation(0, 0, 0.00289794, 0.999996) = Angle: 0.00579588

        initial_poses = []
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -0.868823
        initial_pose.pose.position.y = -0.701858
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
        navigator.clear_periodically_costmap(5)

        navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()
        ######################################################

    def go_to_destination(self):
        navigator = BasicNavigator()

        go = GotoGoal()

        goal_poses = []
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = go.destination_x
        goal_pose.pose.position.y = go.destination_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = go.destination_z
        goal_pose.pose.orientation.w = go.destination_w
        goal_poses.append(goal_pose)
        print(goal_poses)

        # LET'S GO
        nav_start = navigator.get_clock().now()
        # navigator.followWaypoints(goal_poses)
        navigator.goToPose(goal_pose)

        print("GOGOGOGOGOGOGOOGGOOGGO")
        i = 0

        while not navigator.isNavComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    "Distance remaining: "
                    + "{:.2f}".format(feedback.distance_remaining)
                    + " meters."
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=600.0
                ):
                    navigator.cancelNav()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=120.0
                ):
                    goal_pose.pose.position.x = -3.0
                    navigator.goToPose(goal_pose)

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


class BackToStation:
    def back_to_station(self):
        # Launch the ROS 2 Navigation Stack
        navigator = BasicNavigator()

        go = GotoGoal()

        initial_poses = []
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = go.destination_x
        initial_pose.pose.position.y = go.destination_y
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = go.destination_z
        initial_pose.pose.orientation.w = go.destination_w
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
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        goal_poses.append(goal_pose)

        # LET'S GO
        nav_start = navigator.get_clock().now()
        navigator.go_to_pose(goal_poses)

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
                    "Distance remaining: "
                    + "{:.2f}".format(feedback.distance_remaining)
                    + " meters."
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=600.0
                ):
                    navigator.cancelNav()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=120.0
                ):
                    goal_pose.pose.position.x = -3.0
                    navigator.goToPose(goal_pose)

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


class GotoGoal(Node):
    destination_x = 0.0
    destination_y = 0.0
    destination_z = 0.0
    destination_w = 0.0

    def __init__(self):
        super().__init__("Gotogoal_Subscriber")
        self.subscription = self.create_subscription(
            FallDetectionToNav2, "dest_val", self.dest_val_callback, 10
        )
        self.subscription

    def dest_val_callback(self, destination: FallDetectionToNav2):
        self.get_logger().info(
            "Incoming Destination is \nx: %f\n y: %f\n z: %f\n w: %f\n"
            % (
                destination.dest_x,
                destination.dest_y,
                destination.dest_z,
                destination.dest_w,
            )
        )

        GotoGoal.destination_x = destination.dest_x
        GotoGoal.destination_y = destination.dest_y
        GotoGoal.destination_z = destination.dest_z
        GotoGoal.destination_w = destination.dest_w

        start = Go_to_Destination()
        start.go_to_destination()


class Bridge_to_Web_CPR(Node):
    def __init__(self):
        super().__init__("Bridge_Node")

        ## 목적지에 도착 했을 때
        self.publisher = self.create_publisher(Bridge, "arrive_dest", 10)

        # if BasicNavigator.getResult == 1:

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.arrive_callback)
        self.i = 0

        ## CPR, WEB 통신이 끝났을 때
        self.subscription = self.create_subscription(
            Bridge, "situation_end", self.end_callback, 10  # CHANGE
        )
        self.subscription

    def arrive_callback(self):
        msg = Bridge()
        msg.arrive_destination = True
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.arrive_destination)
        self.i += 1

        if self.i == 5:
            exit(0)

    def end_callback(self, msg: Bridge):
        come_back = BackToStation()

        if msg.complete_cpr and msg.complete_web == True:
            come_back.back_to_station()


def main(args=None):
    # Start the ROS 2 Python Client Library
    rclpy.init(args=args)

    # go_to_destination = Go_to_Destination()
    # go_to_destination.set_initial_pose()

    # bridge 노드 실행

    bridge = Bridge_to_Web_CPR()

    # Service 노드 실행

    # go_to_goal = GotoGoal()

    # rclpy.spin(go_to_goal)
    rclpy.spin(bridge)

    # go_to_goal.destroy_node()
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
