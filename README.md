# V-OADRS

[![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://travis-ci.org/SaumilShah66/voadrs.svg?branch=master)](https://travis-ci.org/SaumilShah66/voadrs)
[![Coverage Status](https://coveralls.io/repos/github/SaumilShah66/voadrs/badge.svg?branch=sprint3)](https://coveralls.io/github/SaumilShah66/voadrs?branch=sprint3)
---

## Project Contributors

1) [Saumil Shah](https://github.com/SaumilShah66)
Mechanical Engineer, Innovator, Robotics Graduate Student at UMD. 

2) [Varun Asthana](https://github.com/varunasthana92)
Mechanical Engineer, Innovator, Robotics Graduate Student at UMD. 

## Overview

V-OADRS stands for Vision-based Object Anomaly Detection Robotic System, a special purpose robotic system designed for Acme Robotics, for their future product line. With the changing trend in manufacturing industries for theadoption of autonomous processes, the V-OADRS system can fill the gaps for autonomous inspection toachieve desired quality standards. This robotic system is mainly designed for plastic product manufacturing industries. V-OADRS will scan and inspect the object placed in the defined workspace of the robot, on an assumedwell-defined inspection location or fixture, for proper relative orientation of an object with respect to thearm manipulator (having a scanner mounted on it).
 The system can be programmed to detect any user-defined anomaly (based on the quality needs).  For demonstration, we will detect the presence of a definednumber of holes and their diameters as a quality checkpoint on an object.V-OADRS will generate an output highlighting the number of holes detected and the diameter of each.It will also tell if the diameter of the holes is in the (predefined) acceptable range or not.

## Dependencies

To install and use this package successfully, you need to have all the following dependencies installed on your machine.
* Ubuntu 16.04 LTS
* Ros kinetic
* Gazebo 7.0
* OpenCV 3.3.0
* ROS-Industrial Universal Robot package

You can follow official ubuntu guidelines provided [here](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0) to install Ubuntu 16.04 on your local machine. Though virtual machine can be used, but we highly recommend local installation for better performance with gazebo simulation

Follow official ROS installation guidelines provided [here](http://wiki.ros.org/kinetic/Installation) to install ROS kinetic.

To install gazebo 7.0 follow [this](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install) instructions. 

Install OpenCV 3.3.0 refer [this](https://medium.com/@Linh.NG/installing-opencv-3-3-0-on-ubuntu-16-04-lts-7db376f93961) page.

## Development Process
Agile iterative process is being followed for project. The roles were divided into driver and navigator, which were shuffled as per tasks. You can find all iteration and products logs [here](https://docs.google.com/spreadsheets/d/1BnU5e_QEPwAU8ns-pFhITGS3_m1YnjaZeSdVSFJj3gg/edit#gid=0). Sprint planning and review notes are [here](https://docs.google.com/document/d/1KpbpapvvFNhO2NKPLEjOYNVf_a-0SManJHOhdiiNfMk/edit?usp=sharing).

## How to build

#### Create catkin_ws
You should have a catkin workspace before you can build this project. You can create a catkin workspace with following commands.

```
mkdir catkin_ws && cd catkin_ws
mkdir src && cd src
```
Now if you are present in src directory, you need to clone the official ROS-Industrial Universal Robot package to this workspace and it's dependencies and then clone this repository.

```
sudo apt-get install ros-kinetic-universal-robots
git clone https://github.com/ros-industrial/universal_robot
git clone https://github.com/SaumilShah66/voadrs
```
Official Universal Robot package does not have a camera mounted on it. We have uploaded a modified URDF file with USB camera to this repository and you need to replace official file with the one provided here.
```
cp voadrs/urdf/ur5.urdf.xacro universal_robot/ur_description/urdf/
cp voadrs/urdf/common.gazebo.xacro universal_robot/ur_description/urdf/
```
Now you are ready to build this package. Use following command to build. Make sure you are in your workspace
```
cd catkin_ws
catkin_make
```

## How to launch

You can use following command to launch a demo. This will start a gazebo with robot and environment with inspection parts in it.
```
cd catkin_ws
source devel/setup.bash
roslaunch voadrs voadrs.launch
```
In second terminal
```
cd catkin_ws
source deve/setup.bash
roslaunch voadrs measure.launch
```
The second launch file will ask you which job you want to measure and it will move the robot to that position to measure the dimention of hole.

## Doxygen Documentation
The doxygen generated documents have been added to the docs folder of the repository. A config file named 'Doxyfile' has been added to generate the documentation. To generate the doxygen documentation, follow the steps below:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/turtlebot_navigator/
doxygen Doxyfile
```

## Code Coverage
```
cd ~/catkin_ws/build
lcov --directory . --capture --output-file coverage.info
lcov --list coverage.info
```
This will output the coverage of each file in the terminal. To create an html file for the same, run the following command:
```
genhtml coverage.info --output-directory covout
```
This will store the index.html file in the folder covout.
