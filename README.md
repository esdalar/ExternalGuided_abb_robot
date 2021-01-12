# ExternalGuided_abb_robot
Repository to command an ABB robot or virtual robot controller of RobotStudio with a external simulated sensor using ROS by the interface EGM "External Guided Motion".

![ROS external guiding of a virtual controller robot using EGM](https://github.com/esdalar/ExternalGuided_abb_robot/blob/main/external_guided_app_ROS_RobotStudio.png)

## Installation

Prepare the workspace
-

```
sudo apt-get install ros-melodic-fiducials
```

If the robot is the same IRB6640-185kg/2.8m, just git clone this repo, if not:

```
git clone https://github.com/ros-industrial/abb.git
```
And the choose the robot to use.

********************************

The folowing package will needed to create in a separate workspace to avoid compilation problems:

```
git clone https://github.com/ros-industrial/abb_libegm.git
```




