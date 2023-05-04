#!/usr/bin/env python3

# Author: Bishop Pearson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
  aedbot_imu_parameter = LaunchConfiguration(
    'aedbot_imu_parameter',
    default=os.path.join(
      get_package_share_directory('aedbot_bringup'),
      'param/aedbot_imu.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'aedbot_imu_parameter',
      default_value=aedbot_imu_parameter
    ),

    Node(
      package='aedbot_bringup',
      executable='aedbot_imu_node',
      name='aedbot_imu_node',
      output='screen',
      emulate_tty=True,
      parameters=[aedbot_imu_parameter],
      namespace='',
    )
  ])
