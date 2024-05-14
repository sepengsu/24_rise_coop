# 24_rise_coop
2024년 1학기 RISE COOP 프로젝트

## 기간
2024.03.04 ~ 2024.06.21

## 주제
1. 시뮬레이션 환경 구축
2. SLAM 알고리즘 적용 및 센서 퓨징
3. MPC control

## 
## How to Start
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
1. Install ROS on Remote PC : guidline 3.1.2
2. Install Dependent ROS Pachages : Guidline 3.1.3
3. Install turtleBot3 Backages from source
4. git respositiorty 연결 

## How to install turtleBot3 Backages from source
1. sudo apt remove ros-noetic-dynamixel-sdk
2. sudo apt remove ros-noetic-turtlebot3-msgs
3. sudo apt remove ros-noetic-turtlebot3
4. mkdir -p ~/catkin_ws/src
5. cd ~/catkin_ws/src/
6. git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
7. git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
8. git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
9. git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
10. cd ~/catkin_ws && catkin_make

## 초기 git repostiory 연결 방법 
0. E-manual에서 PC setup 진행 
1. catkin_ws 폴더를 작업폴더로 설정 (git init)
2. git remote add origin https://github.com/sepengsu/24_rise_coop.git
3. git fetch origin
4. git reset --hard origin/main
5. git checkout -b main
6. git push -u origin main 
