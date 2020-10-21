# ROS and RobotRaconteur python turtle module example

## Prerequisite:
* Ubuntu 18.04 ([Native](https://www.linuxtechi.com/ubuntu-18-04-lts-desktop-installation-guide-screenshots/) or [Virtual](https://www.toptechskills.com/linux-tutorials-courses/how-to-install-ubuntu-1804-bionic-virtualbox/))
* python2/python3
* OpenCV (`pip/pip3 install opencv-python`)
* git (`sudo apt-get install git`)
* [RobotRaconteur](https://github.com/robotraconteur/robotraconteur/wiki/Download) (Follow instruction to download)
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (Follow instruction to download)

Clone this repo before continuing the trial:
```
cd ~
git clone https://github.com/hehonglu123/ROS_RR_turtle_trial.git
```
## Basic Examples:
Inside `Example/` folder, there're three basic example scripts, showing how to read keyboard inputs, drive a turtle with python turtle module and detect red object.

### detection.py:
Takes in an image with red circle, outputs a red filtered image.
### keyboard.py
Print command based on arrow key pressed.
### turtlebot.py
Initialize screen display, spawn a turtle and drive the turtle.

## Trial Instruction:
* ROS (https://github.com/hehonglu123/ROS_RR_turtle_trial/blob/master/ROS/src/Trial_instruction.md)
* RR (https://github.com/hehonglu123/ROS_RR_turtle_trial/blob/master/RR/Trial_instruction.md)

![](color_code.gif)
