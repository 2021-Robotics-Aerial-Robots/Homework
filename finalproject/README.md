# drone-gazebo-sim
This is a Gazebo simulation package for ros 18.04. The package is migrated from the [rotorS](https://github.com/ethz-asl/rotors_simulator).

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

```
sudo apt-get install ros-melodic-ompl
sudo apt-get install ros-melodic-mavros
sudo apt-get install ros-melodic-mavros-extras 
sudo apt-get install ros-melodic-mavros-msgs
sudo apt-get install libompl-dev
cd /opt/ros/melodic/lib/mavros
sudo ./install_geographiclib_datasets.sh


```
## The code you should compensate
1. Homework/finalproject/payload/src/ukf.cpp 中有標示 ？ 的地方。
2. Homework/finalproject/ukf/src/ukf.cpp 中有標示 ？ 的地方。

# Compiling
download the package and put it into workspace and use `catkin_make` to build the package.
If the workspace is not ready than try the following command:
```
cd ~/
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/2020-Robotics-Aerial-Robots/Homework.git
cd ..
catkin_make
```
# Running

## Trajecotry tracking
```
roslaunch rotors_gazebo firefly_swarm_hovering_example.launch 
roslaunch rotors_gazebo controller.launch 
roslaunch ukf leader_follower_force_estimate.launch
rosparam set /force_control true
rosparam set /start true

```
## Result
1. The complete simulation result
* [![transportation sim](https://res.cloudinary.com/marcomontalbano/image/upload/v1623296500/video_to_markdown/images/youtube--8W2US6q6Q8E-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/8W2US6q6Q8E "transportation sim")

2. Checking data
* [![transportation plot](https://res.cloudinary.com/marcomontalbano/image/upload/v1623296864/video_to_markdown/images/youtube--uhUozj2F2z4-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/uhUozj2F2z4 "transportation plot")



