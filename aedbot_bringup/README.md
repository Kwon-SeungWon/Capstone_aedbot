# Bringup AEDBOT
> AEDBOT의 Motor, Imu, Lidar 노드를 실행하고 EKF filter를 통해 Motor의 엔코더 값과 Imu Sensor의 값을 Calibration하여 odom값을 내보낸다.


## AEDBOT bringup
```bash
cd {$workspace_path}
ros2 launch aedbot_bringup aedbot_bringup.launch.py
```
- rqt_graph
```bash
ros2 run rqt_graph rqt_graph
```
![image](https://github.com/Kwon-SeungWon/Capstone_aedbot/assets/99318074/f9f0fdd6-ac28-4427-8567-54d94ab55721)
##
