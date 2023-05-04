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
import launch_ros.actions

def generate_launch_description():

  aedbot_mcu_parameter = LaunchConfiguration(
    'aedbot_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('aedbot_bringup'),
      'param/aedbot_mcu.yaml'
    )
  )

  aedbot_lidar_parameter = LaunchConfiguration(
    'aedbot_lidar_parameter',
    default=os.path.join(
      get_package_share_directory('aedbot_bringup'),
      'param/aedbot_lidar.yaml'
    )
  )

  aedbot_imu_parameter = LaunchConfiguration(
    'aedbot_imu_parameter',
    default=os.path.join(
      get_package_share_directory('aedbot_bringup'),
      'param/aedbot_imu.yaml'
    )
  )
  
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  aedbot_description_dir = LaunchConfiguration(
    'aedbot_description_dir',
    default=os.path.join(
      get_package_share_directory('aedbot_description'),
      'launch'
    )
  )
  
  return LaunchDescription([
    
    # EKF Filter 적용
    launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("aedbot_bringup"), 'param', 'ekf.yaml')],
    ),

    DeclareLaunchArgument(
      'aedbot_mcu_parameter',
      default_value=aedbot_mcu_parameter
    ),

    DeclareLaunchArgument(
      'aedbot_lidar_parameter',
      default_value=aedbot_lidar_parameter
    ),

    DeclareLaunchArgument(
      'aedbot_imu_parameter',
      default_value=aedbot_imu_parameter
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/aedbot_mcu.launch.py']),
      launch_arguments={'aedbot_mcu_parameter': aedbot_mcu_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/aedbot_lidar.launch.py']),
      launch_arguments={'aedbot_lidar_parameter': aedbot_lidar_parameter}.items()
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/aedbot_imu.launch.py']),
      launch_arguments={'aedbot_imu_parameter': aedbot_imu_parameter}.items()
    ),
    
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([aedbot_description_dir, '/aedbot_state_publisher.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),
  ])
