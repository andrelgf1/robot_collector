# Project WICKMAN - ROBOT COLLECTOR for ACME Robotics

[![Build Status](https://travis-ci.org/sanhuezapablo/robot_collector.svg?branch=master)](https://travis-ci.org/sanhuezapablo/robot_collector)
[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
---

## Overview



## Personnel

Andre Gomes - Graduated in computer engineering at Uniceub in Brazil.

Ryan Cunningham - Works for Booz Allen Hamilton as a full-stack software engineer. Graduated with a Computer Science degree from UMBC in 2013.

Pablo - Graduated with a B.S. in Mechanical Engineering from UMCP in 2017. Currently pursuing a M. Eng. in Robotics at UMCP. Highly interested in Machine Learning and Autonomous Robots. 


## License

MPL 2.0 License
Copyright 2019 Ryan Cunningham, Andre Gomes, Pablo Sanhueza

```
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

```


## AIP Google Sheet

https://drive.google.com/file/d/1VDC9aNsyO4hMGFzV3Twza4A3Bski4WT-/view?usp=sharing


## Review of Iteration 1 Plan



## Dependencies

This repo is intended to be used on a device with Ubuntu 16.04 LTS. Additionally, you need to have ROS Kinetic Kame installed, which comes with Gazebo (we will also use this). Finally, you will also need the Turtlebot simulation stack. The list below are things that you will need to build and use this program.

- Install ROS Kinetic. Make sure you follow all steps. Please follow link [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- Create a catkin Package. Catkin needs to be installed. Follow link [here](http://wiki.ros.org/catkin)

- Install Turtlebot simulation stack. Follow commands shown below.

```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
$ source /opt/ros/kinetic/setup.bash
```



## Build Intructions

To run this code you will need catkin. Also, you will need to have your catkin workspace set up. Follow commands below.

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ cd src/
$ git clone -b https://github.com/sanhuezapablo/robot_collector.git
$ cd ..
$ catkin_make
```

If you do not have a catkin workspace set-up, please follow commands below. **Disregard if you already have catkin workspace.**
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone -b https://github.com/sanhuezapablo/robot_collector.git
$ cd ..
$ catkin_make
```


## Building for code coverage


## How to run demo


## How to run tests


## Known Issues/Bugs


### How to Generate Doxygen Documentation 

How to install Doxygen:

```
sudo apt-get install doxygen
```

Inside the cloned Directory, enter the following terminal commands.

```
 doxygen Doxyfile
```

Opening Doxygen Documentation in Firefox
```
 cd Docs
 cd html
 firefox index.html
```


## Plugins

- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Git

    It is possible to manage version control through Eclipse and the git plugin, but it typically requires creating another project. If you're interested in this, try it out yourself and contact me on Canvas.
