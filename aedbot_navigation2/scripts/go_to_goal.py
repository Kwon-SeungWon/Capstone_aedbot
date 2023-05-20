#! /usr/bin/env python3
import time  # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from robot_navigator import BasicNavigator, NavigationResult # Helper module
 
'''
Navigates a robot from an initial pose to a goal pose.
'''
def main():
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
 
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  # Set the robot's initial pose if necessary
  initial_pose = PoseStamped()
  initial_pose.header.frame_id = 'map'
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
  #navigator.lifecycleStartup()
 
  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()
 
  # If desired, you can change or load the map as well
  # navigator.changeMap('/path/to/map.yaml')
 
  # You may use the navigator to clear or obtain costmaps
  navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  # global_costmap = navigator.getGlobalCostmap()
  # local_costmap = navigator.getLocalCostmap()
 
  # Set the robot's goal pose
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 3.0
  goal_pose.pose.position.y = 0.2
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
 
  # sanity check a valid path exists
  # path = navigator.getPath(initial_pose, goal_pose)
 
  # Go to the goal pose
  navigator.goToPose(goal_pose)
 
  i = 0
 
  # Keep doing stuff as long as the robot is moving towards the goal
  while not navigator.isNavComplete():
    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################
    #navigator.clear_periodically_costmap()
    print('clear')
    # Do something with the feedback
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Distance remaining: ' + '{:.2f}'.format(
            feedback.distance_remaining) + ' meters.')
 
      # Some navigation timeout to demo cancellation
      if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
        navigator.cancelNav()
 
      # Some navigation request change to demo preemption
      if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
        goal_pose.pose.position.x = -3.0
        navigator.goToPose(goal_pose)
 
  # Do something depending on the return code
  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
      print('Goal succeeded!')
      print('dfasddddddddddddddddddddddddd')
  elif result == NavigationResult.CANCELED:
      print('Goal was canceled!')
  elif result == NavigationResult.FAILED:
      print('Goal failed!')
  else:
      print('Goal has an invalid return status!')
 
  # Shut down the ROS 2 Navigation Stack
  navigator.lifecycleShutdown()
 
  exit(0)
 
if __name__ == '__main__':
  main()