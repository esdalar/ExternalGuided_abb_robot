# ExternalGuided_abb_robot
Repository to command an ABB robot or virtual robot controller of RobotStudio with a external simulated sensor using ROS by the interface EGM "External Guided Motion".

Take into account that the EGM only runs from RobotWare version 6. 

![ROS external guiding of a virtual controller robot using EGM](https://github.com/esdalar/ExternalGuided_abb_robot/blob/main/external_guided_app_ROS_RobotStudio.png)

## Installation 🔧

_Create 2 workspace: one for the application core and the other for "abb_libegm" package_

```
$ mkdir -p ~/catkin_wsApp/src
$ cd ~/catkin_wsApp/
$ catkin_make
```

 📌 _The folowing package is needed to create in a separate workspace to avoid compilation problems:_

```
$ mkdir -p ~/catkin_wsLibEgm/src
$ cd ~/catkin_wsLibEgm/
$ catkin_make

$ git clone https://github.com/ros-industrial/abb_libegm.git
$ catkin_make_isolated --pkg abb_libegm --only-pkg-with-deps abb_libegm --install
```
_Install the packages for the application_

```
$ sudo apt-get install ros-melodic-fiducials
$ catkin_make_isolated --pkg egm_interface
$ catkin_make
$ cd devel/setup.bash

```

## Custom configuration ⚙️

_If the robot is the same IRB6640-185kg/2.8m, just git clone this repo_
_if no,  choose the robot to use._


```
$ git clone https://github.com/ros-industrial/abb.git
```

## Connect to the real robot or Virtual controller of RobotStudio and run the application  🚀
```
$ roslaunch project simulation.launch
$ rosrun egm_interface egm_interface
$ rosrun egm_path egm_path

```



