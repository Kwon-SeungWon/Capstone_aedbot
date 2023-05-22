#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import os

def main():
    #os.system("arduino")
    os.system("ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0")  

if __name__ == "__main__":
    main()