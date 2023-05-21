#! /usr/bin/env python3
import time  # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node

from aedbot_interfaces.msg import Bridge, FallDetectionToNav2
from robot_navigator import BasicNavigator, NavigationResult  # Helper module
from std_msgs.msg import Int32
 
'''
Navigates a robot from an initial pose to a goal pose.
'''


class Sub_dest_val_Go_to_Destination(Node):
    flag = 0
    destination_x = 0.0
    destination_y = 0.0
    destination_z = 0.0
    destination_w = 0.0

    def __init__(self):
        super().__init__("Sub_dest_val_Go_to_Destination")
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

        Sub_dest_val_Go_to_Destination.destination_x = destination.dest_x
        Sub_dest_val_Go_to_Destination.destination_y = destination.dest_y
        Sub_dest_val_Go_to_Destination.destination_z = destination.dest_z
        Sub_dest_val_Go_to_Destination.destination_w = destination.dest_w

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
        navigator.clearCostmapsPeriodically(3)
        # Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = destination.dest_x
        goal_pose.pose.position.y = destination.dest_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = destination.dest_z
        goal_pose.pose.orientation.w = destination.dest_w
      
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
            Sub_dest_val_Go_to_Destination.flag = 1
            print(Sub_dest_val_Go_to_Destination.flag)
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
      
        # Shut down the ROS 2 Navigation Stack
        navigator.lifecycleShutdown()

class Bridge_to_Web_CPR(Node):
    def __init__(self):
        
        super().__init__("Bridge_Node")

        ## 목적지에 도착 했을 때
        self.publisher = self.create_publisher(Bridge, "arrive_dest", 10)

        ## CPR, WEB 통신이 끝났을 때
        self.subscription = self.create_subscription(
            Int32, "situation_end", self.end_callback, 10  # CHANGE
        )
        self.subscription

        ## 목적지 도착 후 flag = 1 상태에서 arrive_callback 1초마다 호출
        if Sub_dest_val_Go_to_Destination.flag == 1:
            print("162 flag")
            timer_period = 1
            self.timer = self.create_timer(timer_period, self.arrive_callback)
        

    def arrive_callback(self):
        global flag
        print("arrive callback")
        msg = Bridge()
        msg.arrive_destination = True
        self.publisher.publish(msg)
        
        if Sub_dest_val_Go_to_Destination.flag == 2:
            msg.arrive_destination = False
            self.publisher.publish(msg)

        self.get_logger().info('Publishing: "%d"' % msg.arrive_destination)


        # if self.i == 5:
        #     exit(0)

    def end_callback(self, msg: Int32):

        if msg.data == 1:
            Sub_dest_val_Go_to_Destination.flag = 2 
            # Launch the ROS 2 Navigation Stack
            navigator = BasicNavigator()
          
            # Set the robot's initial pose if necessary
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = Sub_dest_val_Go_to_Destination.destination_x
            initial_pose.pose.position.y = Sub_dest_val_Go_to_Destination.destination_y
            initial_pose.pose.position.z = 0.0
            initial_pose.pose.orientation.x = 0.0
            initial_pose.pose.orientation.y = 0.0
            initial_pose.pose.orientation.z = Sub_dest_val_Go_to_Destination.destination_z
            initial_pose.pose.orientation.w = Sub_dest_val_Go_to_Destination.destination_w
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
            #navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
            # global_costmap = navigator.getGlobalCostmap()
            # local_costmap = navigator.getLocalCostmap()
            navigator.clearCostmapsPeriodically(3)
            # Set the robot's goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = 0.0
            goal_pose.pose.position.y = 0.0
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
                flag = 3
                print('ENDENDENDENDENDENDENDEND')
            elif result == NavigationResult.CANCELED:
                print('Goal was canceled!')
            elif result == NavigationResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
          
            # Shut down the ROS 2 Navigation Stack
            navigator.lifecycleShutdown()
          


def main(args=None):
    # Start the ROS 2 Python Client Library
    rclpy.init(args=args)

    sub_dest_val_go_to_destination = Sub_dest_val_Go_to_Destination()
    bridge = Bridge_to_Web_CPR()

    rclpy.spin(sub_dest_val_go_to_destination)
    rclpy.spin(bridge)
    
    sub_dest_val_go_to_destination.destroy_node()
    bridge.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
  main()