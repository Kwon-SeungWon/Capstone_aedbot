# AEDBOT_CAPSTONE Project (in ROS2 FOXY) 

This project is to demonstrate  and navigation in ROS2-foxy environment.  

## Build source

### Docker Pull

Docker image를 Pull하면 된다.
https://hub.docker.com/layers/kwonseungwon/ros2/capstone_0509/images/sha256-7b6132c6f8086dc61289dfc6ad97402be93093bca1b463ccdaeb2d460e9743f1?context=repo

Docker image Environment : ros2-foxy

```bash
  docker pull kwonseungwon/ros2:capstone_0509
```

### Run Docker Container from Image

Pull한 Docker의 image를 컨테이너로 실행시킨다.

```bash
  sudo docker run --net=host --env="DISPLAY" --volume="/dev:/dev" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -it --name 'Capstone_AEDBOT' --privileged kwonseungwon/ros2:capstone_0509 /bin/bash 
```

Gazebo가 GUI를 필요로하기에, Docker에서의 외부출력을 위해 docker option을 추가했다. (--env)
—name 뒤에는 ‘’안에 자기가 만들고 싶은 컨테이너 이름을 넣으면 된다.('Capstone_AEDBOT')
—privileged 은 컨테이너를 Privileged 모드로 실행하면 시스템의 **모든 장치에 접근할 수 있으며 커널의 기능을 대부분 사용**할 수 있다.
--volume="/dev:/dev" 은 host의 컴퓨터와 환경을 동일시하여 USB 포트인식 및 추후 포트이름을 변경할 수 있다.

### Enter Docker Container

```bash
  xhost + # Available Display GUI
  docker start Capstone_AEDBOT
  docker exec -it Capstone_AEDBOT bash
```

### Build ROS2 source

- To build

```bash
  cd {$workspace_path}
  colcon build --symlink-install
```

- To enable the built source into ROS2 environment

```bash
  cd {$workspace_path}
  ./install/setup.bash
```

- To change USB PORT NAME

```bash
  cd /etc/udev/rules.d
  sudo gedit 99-tty.rules
```
```bash
  KERNEL=="tty*", SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyLIDAR"
KERNEL=="tty*",SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0002", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyIMU"
KERNEL=="tty*",SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0003", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyIMU2"
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", SYMLINK+="ttyMotor"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="003d", ATTRS{serial}=="14235303036351707242", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyArduino"
```

만약 rules.d 안에 다른 파일들이 있으면 파일 이름 뒤에 .backup을 붙여놓자.

- Udevadm rules restart

```bash
  sudo service udev reload
sudo service udev restart
```
명령어 입력 후 장치를 제거하고 다시 연결한다.

## Start AEDBOT

- To bringup robot

```bash
cd {$workspace_path}
ros2 launch aedbot_bringup aedbot_bringup.launch.py
```

- To bringup robot (in **simulation environment**)
```bash
cd {$workspace_path}
ros2 launch omo_r1mini_gazebo omo_r1mini.launch.py
```
- To teleoperate the robot using **KEYBOARD**

```bash
cd {$workspace_path}
ros2 run aedbot_teleop teleop_keyboard
```

- To conduct SLAM (Try after few seconds from bringup)

```bash
cd {$workspace_path}
ros2 launch aedbot_cartographer cartographer.launch.py
```
```bash
cd {$workspace_path}
ros2 launch aedbot_cartographer cartographer_rviz.launch.py
```

- Once mapping is done, you can create map.pgm and map.yaml file by executing

```bash
cd {$HOME}
ros2 run nav2_map_server map_saver_cli -f map
```

- To conduct path planning & following (Try after few seconds from bringup)
```bash
cd {$workspace_path}
ros2 launch aedbot_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

```bash
ros2 launch aedbot_navigation2 navigation2_rviz.launch.py
```
