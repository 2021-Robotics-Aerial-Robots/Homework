# multirotor_geometry_control
* This is a Gazebo simulation package migrated from the [rotorS](https://github.com/ethz-asl/rotors_simulator) for ros-melodic (Ubuntu 18.04).
* The geometry controller of the multirotor is implemented in this simulation.
* The parameters of the multirotor(firefly model) can be found in /multirotor_geometry_control/rotors_simulator/rotors_description/urdf/firefly.xacro.
* Gains used for geometric control(firefly model) are defined in /multirotor_geometry_control/rotors_simulator/rotors_gazebo/resource/lee_controller_firefly.yaml.

## Requirements
* Ubuntu 18.04 ros-melodic
* Gazebo 9 simulator

```
sudo apt-get install ros-melodic-joy
sudo apt-get install ros-melodic-octomap-ros
sudo apt-get install ros-melodic-mavlink
sudo apt-get install python-wstool
sudo apt-get install python-catkin-tools
sudo apt-get install protobuf-compiler
sudo apt-get install libgoogle-glog-dev
sudo apt-get install ros-melodic-control-toolbox
```

## Usage
* Use firefly as the simulation model
```
roslaunch rotors_gazebo firefly_one.launch
roslaunch rotors_gazebo controller_geometry_firefly.launch
```

* Use iris as the simulation model
```
roslaunch rotors_gazebo iris_one.launch
roslaunch rotors_gazebo controller_geometry_iris.launch
```

