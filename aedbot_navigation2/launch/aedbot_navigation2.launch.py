# #!/usr/bin/env python3

import os
import launch
import launch.actions
import launch.substitutions
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    package_name = "aedbot_navigation2"  # 실행하려는 패키지의 이름으로 변경해주세요

    # 각 런치 파일을 실행하는 액션을 생성합니다
    nav2_launch = launch.actions.ExecuteProcess(
        cmd=["ros2", "launch", package_name, "navigation2.launch.py"], output="screen"
    )

    nav2_rviz_launch = launch.actions.ExecuteProcess(
        cmd=["ros2", "launch", package_name, "navigation2_rviz.launch.py"],
        output="screen",
    )

    go_to_goal = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                os.getcwd(), "src", "aedbot", package_name, "scripts", "go_to_goal.py"
            )
        ],
        output="screen",
    )

    test_pub = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                os.getcwd(), "src", "aedbot", "aedbot_nodes", "scripts", "test_pub_dest"
            )
        ],
        output="screen",
    )

    # TimerAction을 사용하여 시간 간격을 두고 nav2_rviz_launch를 실행합니다.
    timer_action = launch.actions.TimerAction(
        period=3.0,  # 시간 간격을 설정합니다. 여기서는 5초로 설정했습니다.
        actions=[nav2_rviz_launch],  # 실행할 액션을 리스트로 전달합니다.
    )

    timer_action2 = launch.actions.TimerAction(
        period=2.0,  # 두 번째 액션(file2_launch)의 시간 간격을 설정합니다. 여기서는 10초로 설정했습니다.
        actions=[go_to_goal],  # 두 번째 액션을 실행합니다.
    )

    timer_action3 = launch.actions.TimerAction(
        period=4.0,  # 두 번째 액션(file2_launch)의 시간 간격을 설정합니다. 여기서는 10초로 설정했습니다.
        actions=[test_pub],  # 두 번째 액션을 실행합니다.
    )

    return launch.LaunchDescription(
        [
            nav2_launch,
            timer_action,
            timer_action2
            # timer_action3
        ]
    )


if __name__ == "__main__":
    launch_description = generate_launch_description()
    launch.launch(launch_description)

# import os
# import launch
# import launch.actions
# import launch.substitutions
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import launch_ros.actions
# import launch.event_handlers

# def generate_launch_description():
#     package_name = 'aedbot/aedbot_navigation2'  # 패키지 이름을 여기에 입력해주세요

#     # 각 런치 파일을 실행하는 액션을 생성합니다
#     nav2_launch = launch.actions.IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(os.getcwd(), 'src', package_name, 'launch', 'navigation2.launch.py')
#         )
#     )

#     nav2_rviz_launch = launch.actions.IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(os.getcwd(), 'src', package_name, 'launch', 'navigation2_rviz.launch.py')
#         )
#     )

#     go_to_goal = launch.actions.IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(os.getcwd(), 'src', package_name, 'scripts', 'go_to_goal.py')
#         )
#     )

#     return launch.LaunchDescription([
#         nav2_launch,
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessStart(
#                 target_action=nav2_launch,
#                 on_start=[nav2_rviz_launch]
#             )
#         )
#     ])


# if __name__ == '__main__':
#     generate_launch_description()
