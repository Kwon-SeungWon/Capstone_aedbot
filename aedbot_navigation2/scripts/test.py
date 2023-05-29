#! /usr/bin/env python3
import time  # Time library

from geometry_msgs.msg import PoseStamped  # Pose with ref frame and timestamp
from rclpy.duration import Duration  # Handles time for ROS 2
import rclpy  # Python client library for ROS 2
from rclpy.node import Node

from aedbot_interfaces.msg import Bridge, FallDetectionToNav2
from robot_navigator import BasicNavigator, NavigationResult  # Helper module
from std_msgs.msg import Int32
import requests

"""
Navigates a robot from an initial pose to a goal pose.
"""

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = -0.7085
    initial_pose.pose.orientation.w = 0.705619
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = -0.7085
    goal_pose.pose.orientation.w = 0.705

    navigator.goToPose(goal_pose)
    
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
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print("Goal succeeded!")

        msg = Bridge()
        msg.nav2_to_bridge = True
        self.publisher_.publish(msg)
        requests.get("http://130.162.152.119/arrive")

        print(msg)
        navigator.lifecycleShutdown()
        self.should_exit = True

    elif result == NavigationResult.CANCELED:
        print("Goal was canceled!")
    elif result == NavigationResult.FAILED:
        print("Goal failed!")
    else:
        print("Goal has an invalid return status!")

    exit(0)

if __name__ == "__main__":
    main()