#! /usr/bin/env python3

import time  # Time library

from geometry_msgs.msg import PoseStamped  # Pose with ref frame and timestamp
from rclpy.duration import Duration  # Handles time for ROS 2
import rclpy  # Python client library for ROS 2
from rclpy.node import Node

from aedbot_interfaces.srv import FallDetectionToNav2
from robot_navigator import BasicNavigator, NavigationResult  # Helper module

"""
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
"""


class GotoGoal(Node):
    def __init__(self):
        super().__init__("Gotogoal_Server")
        self.srv = self.create_service(
            FallDetectionToNav2, "dest_val", self.dest_val_callback
        )

    def dest_val_callback(self, destination, location):
        goal_poses = []
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = BasicNavigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = destination.dest_x
        goal_pose.pose.position.y = destination.dest_y
        goal_pose.pose.orientation.z = destination.dest_z
        goal_pose.pose.orientation.w = destination.dest_w
        self.goal = goal_poses.append(goal_pose)
        self.get_logger().info(
            "Incoming Destination is \nx: %f\n y: %f\n z: %f\n w: %f\n"
            % (
                destination.dest_x,
                destination.dest_y,
                destination.dest_z,
                destination.dest_w,
            )
        )
        location = "사고 발생지점 수신완료"
        
        self.go_to_destination()

        return location

    def go_to_destination(self):
        # Launch the ROS 2 Navigation Stack
        navigator = BasicNavigator()

        # Set the robot's initial pose if necessary
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

        # LET'S GO
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(self.goal)

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


def main():
    # Start the ROS 2 Python Client Library
    rclpy.init()

    # Service 노드 실행
    go_to_goal_service = GotoGoal()

    rclpy.spin(go_to_goal_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
