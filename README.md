
# Drone Lane Tracking
육상 트랙위를 달리는 선수를 따라가며 촬영하는 자율 주행하는 드론입니다.

전면 카메라에서는 트랙선을 탐지하여 경로를 제어하고, 측면 카메라에서는 선수의 위치를 탐지하여 속도를 제어합니다.

[DQLL](https://github.com/tuzixini/DQLL)이란 모델을 참고하여 탐지 모델을 설계했습니다. [[link](https://github.com/jsh0551/DQLL)]<br/><br/>

- Simulation(AIRSIM)

![airsim1](https://github.com/user-attachments/assets/ff4cc966-67fb-4d9a-8a1a-67b0a24d85d5)
![airsim2](https://github.com/user-attachments/assets/113ed5ed-a5c0-4a8f-8402-6b0c0890d37d)

- Field test

![100](https://github.com/user-attachments/assets/9e8f0907-f1a1-4c30-bf02-cc54693dbe16)
![200](https://github.com/user-attachments/assets/4a4e9755-dd63-4dad-81f9-67d66111fd3d)


## Prerequisites
- Simulation (Windows11)
	- Unreal Engine (4.27.2)
		- Library : [Olympics Athletics Stadium](https://www.fab.com/ko/listings/38ebbdc2-d89f-4a20-8057-5ff228b7f778)
	- [AIRSIM](https://microsoft.github.io/AirSim/build_windows/)
 - Hardware
   - single board computer : NVIDIA Jetson Orin NX
     - [Jetpack 6.0](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
   - flight controller : Holybro Pixhawk 6C Mini
   - camera : oCam-5CRO-U-M
   - LIDAR : Benewake TFmini Plus
- OS
  - version : Ubuntu-22.04
  - cuda-toolkit : 11.8
- ROS Version
	- [ROS2-humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#install-ros-2-packages)
- Requirements
  - torch==2.2.0
  - torchaudio==2.2.0
  - torchvision==0.17.0
  - mmcv-full==1.7.2
  - opencv-python==4.9.0.80
  - opencv-contrib-python==4.9.0.80
  - ultralytics
  - easydict
   
## Installation (AIRSIM)
### Windows11

```
cd /path/to/
pip install airsim
```



### WSL2
```
git clone https://github.com/jsh0551/drone-lane-tracking.git
cd drone-lane-tracking
colcon build
source install/setup.bash
```
- [read_camera_windows11.py](https://github.com/jsh0551/drone-lane-tracking/blob/main/read_camera_windows11.py "read_camera_windows11.py") 파일은 Windows11 워크스페이스로 이동

1. PX4 설치
```
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo reboot
```
2. AIRSIM과 PX4 연동
https://www.youtube.com/watch?v=e3HUKGAWdx0&t=1160s

3. MAVROS
```
# install mavros
sudo apt-get install ros-${ROSDISTRO}-mavros ros-${ROSDISTRO}-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```


## Getting Started

### Running launch file

- AIRSIM
  - WSL2에서 px4와 mavros를 실행하여 airsim과 연결 후 launch 파일 실행
    ```
    # in WSL2
    ## terminal 1
    cd ~/PX4-Autopilot
    make px4_sitl_default none_iris

    ## terminal 2
    ros2 launch mavros px4.launch fcu_url:=udp://:14030@127.0.0.1:14280

    ## terminal 3
    ros2 launch auto_drone auto_drone_airsim_launch.py
    ```
    
  - airsim 이미지 정보를 TCP로 WSL2에 실시간 전송
 
    ```
    # in Windows11
    cd /path/to/
    python3 read_camera_windows11.py
    ```

- Field test
  - pixhawk 연결된 usb 포트에 권한 부여 후 mavros 실행
    ```
    sudo chmod a+rw /dev/ttyUSB0
    ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyUSB0:57600 --params-file ./px4_config.yaml
    ```
  - launch 파일 실행
    ```
    ros2 launch auto_drone auto_drone_launch.py
    ```
    
### Command

1. 이륙
```
# in WSL2
## command terminal
ros2 run auto_drone takeoff
```
2-1. 주행 (recording)
```
# in WSL2
## command terminal
ros2 run auto_drone drive
```
2-2. 주행 (not recording)
```
# in WSL2
## command terminal
ros2 run auto_drone drive_auto
```

3. 착륙
```
# in WSL2
## terminal 4
ros2 run auto_drone land
```
