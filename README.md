# ExternalGuided_abb_robot
Repository to command an ABB robot or virtual robot controller of RobotStudio with a external simulated sensor using ROS by the interface EGM "External Guided Motion".

Take into account that the EGM only runs from RobotWare version 6. 

![ROS external guiding of a virtual controller robot using EGM](https://github.com/esdalar/ExternalGuided_abb_robot/blob/main/external_guided_app_ROS_RobotStudio.png)

## Installation 🔧

Create the workspace to work in

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

_Install the packages to be use in your workspace_

```
sudo apt-get install ros-melodic-fiducials
```
## Custom configuration ⚙️

_If the robot is the same IRB6640-185kg/2.8m, just git clone this repo_
_if no,  choose the robot to use_ .
_

```
git clone https://github.com/ros-industrial/abb.git
```

********************************

 📌 The folowing package will needed to create in a separate workspace to avoid compilation problems:

```
git clone https://github.com/ros-industrial/abb_libegm.git
```




