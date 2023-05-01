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

  omo_r1mini_mcu_parameter = LaunchConfiguration(
    'omo_r1mini_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('omo_r1mini_bringup'),
      'param/omo_r1mini_mcu.yaml'
    )
  )

  omo_r1mini_lidar_parameter = LaunchConfiguration(
    'omo_r1mini_lidar_parameter',
    default=os.path.join(
      get_package_share_directory('omo_r1mini_bringup'),
      'param/omo_r1mini_lidar.yaml'
    )
  )

  omo_r1mini_imu_parameter = LaunchConfiguration(
    'omo_r1mini_imu_parameter',
    default=os.path.join(
      get_package_share_directory('omo_r1mini_bringup'),
      'param/omo_r1mini_imu.yaml'
    )
  )
  
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  omo_r1_description_dir = LaunchConfiguration(
    'omo_r1_description_dir',
    default=os.path.join(
      get_package_share_directory('omo_r1_description'),
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
            parameters=[os.path.join(get_package_share_directory("omo_r1mini_bringup"), 'param', 'ekf.yaml')],
    ),

    DeclareLaunchArgument(
      'omo_r1mini_mcu_parameter',
      default_value=omo_r1mini_mcu_parameter
    ),

    DeclareLaunchArgument(
      'omo_r1mini_lidar_parameter',
      default_value=omo_r1mini_lidar_parameter
    ),

    DeclareLaunchArgument(
      'omo_r1mini_imu_parameter',
      default_value=omo_r1mini_imu_parameter
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1mini_mcu.launch.py']),
      launch_arguments={'omo_r1mini_mcu_parameter': omo_r1mini_mcu_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1mini_lidar.launch.py']),
      launch_arguments={'omo_r1mini_lidar_parameter': omo_r1mini_lidar_parameter}.items()
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1mini_imu.launch.py']),
      launch_arguments={'omo_r1mini_imu_parameter': omo_r1mini_imu_parameter}.items()
    ),
    
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([omo_r1_description_dir, '/omo_r1_state_publisher.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),
  ])
