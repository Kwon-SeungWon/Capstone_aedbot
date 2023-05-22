#!/usr/bin/env python3


#######################################
### This is for BringUp aedbot_nodes###
#######################################

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
import launch_ros.actions

def generate_launch_description():
    package_name = 'aedbot_nodes'  # 패키지 이름을 여기에 입력해주세요

    actions = []

    # 특정 Python 파일들의 실행을 추가합니다.

    # actions.append(launch_ros.actions.Node(
    #     package=package_name,
    #     executable='cpr_node',
    #     output='screen',
    #     name='cpr_node'
    # ))

    # actions.append(launch_ros.actions.Node(
    #     package=package_name,
    #     executable='get_dest',
    #     output='screen',
    #     name='get_dest'
    # ))

    # actions.append(launch_ros.actions.Node(
    #     package=package_name,
    #     executable='facetime',
    #     output='screen',
    #     name='facetime'
    # ))

    # actions.append(launch_ros.actions.Node(
    #     package=package_name,
    #     executable='bridge_node',
    #     output='screen',
    #     name='bridge_node'
    # ))

    actions.append(launch_ros.actions.Node(
        package=package_name,
        executable='HRI',
        output='screen',
        name='HRI'
    ))

    actions.append(launch_ros.actions.Node(
        package=package_name,
        executable='imu_subscriber',
        output='screen',
        name='imu_subscriber'
    ))

    actions.append(launch_ros.actions.Node(
        package=package_name,
        executable='play_siren',
        output='screen',
        name='play_siren'
    ))

    # 추가적인 파일들의 실행을 원한다면 위의 형식으로 계속 추가합니다.

    return launch.LaunchDescription(actions)


if __name__ == '__main__':
    generate_launch_description()