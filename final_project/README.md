# Final Project

Implementation Issue:
https://hackmd.io/@ncrl-10/SkwUc5QH9

The packages for 2021-AerialRobotics Final Project.

1. Rotor_simulator : UAV simulation using Geometric Controller

2. Husky : Ground Vehicle for assisting UAV challenge
    - Navigation
    - Localization
    - SLAM

3. Apriltag : For localization



# Requirements
* Ubuntu 18.04 ros-melodic
* gazebo greater than 9.0

```
sudo apt-get install ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
sudo apt-get install python-cvxopt
//or
sudo pip install cvxopt
```
# Additional package

## Rotor_simulator
This is a Gazebo simulation package for ros 18.04. The package is migrated from the [rotorS](https://github.com/ethz-asl/rotors_simulator).
```
sudo apt-get install ros-melodic-ompl
sudo apt-get install ros-melodic-mavros
sudo apt-get install ros-melodic-mavros-extras 
sudo apt-get install ros-melodic-mavros-msgs
sudo apt-get install libompl-dev
cd /opt/ros/melodic/lib/mavros
sudo ./install_geographiclib_datasets.sh
```

## Husky
```
sudo apt-get install ros-melodic-husky-simulator
```
## Apriltag
```
sudo apt-get install ros-melodic-apriltag-ros
cd apriltag
cmake .
sudo make install
cd ..

# move apriltag installation folder to another folder to avoid error
mv apriltag/ ~/
```



# Running

## Setup
```
# if you don't have one, try to review ros architecture  and create one
cd ~/catkin_ws/src

git clone git@github.com:2021-Robotics-Aerial-Robots/Homework.git
mv /Homework/final_project final_project

# delete Homework file, you will need to enter the password
sudo rm -r /Homework

cd ~/catkin_ws

# it will take about 2-5 minutes for first time
catkin_make

# Remember if you cannot find launch file or node file, execute this command! 
source ~/catkin_ws/devel/setup.bash
```
## Spawn husky and quadcopter

```
roslaunch rotors_gazebo challenge.launch 
```

![](https://i.imgur.com/NNoYdvO.png)

## Run position controller 
```
roslaunch rotors_gazebo control_challenge.launch 
```
![](https://i.imgur.com/M255jPo.png)



# Challenge 1

## Apriltag tutorial
https://blog.csdn.net/wangmj_hdu/article/details/112668252
## Run Apriltag_detector
```
roslaunch apriltag_ros continuous_detection.launch
rostopic echo /tag_detections
```
![](https://i.imgur.com/8Ptwd8p.png)

