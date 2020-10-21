# Robotic Arm Push Lab 


## Description:
   Robot simulation software is making robotic automation safer, less risky, and much more obtainable. In this lab, you will be introduced to the basics of creating a robotic arm using ROS tools such as Rviz and Nodes, and Gazebo; a 3D dynamic simulator that can accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. First, check you have the prerequisites in place, then you can direct to the introduction and get the lab started. 

## Prerequisites: 
* Ubuntu 18.04: https://releases.ubuntu.com/18.04/ (The Desktop Server is Recommended)
* python2 or python3
* Git (`sudo apt-get install git`)
* ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu (Follow the instructions to download)

## Introduction:
  Here, you are going to design a Robotic Arm Manipulator from scratch, implementing the various elements and attributes to it, and move the robot using ROS topics and rqt. ROS stands for the Robot Operating System. It is an open source robotics middleware which consists of numerous software libraries and tools that will aid you in building robot applications. From drivers to state of the art algorithms and simulations, ROS encompasses all the needs to build a robotics project. You can install ROS directly on your machine, but it is highly recommended to do so on a virtual machine. Through a virtual machine such as: WorkstationPro for Windows, VMware Fusion for MacOS, or freeware VirtualBox available for both but with a bit lower quality, install Ubuntu 18.04 Bionic full-desktop (Ubuntu 18.04.5 LTS (Bionic Beaver). Inside the linux distribution, Ubuntu, use the following link (Melodic/Install - ROS Wiki) to install ROS melodic.
  After having the prerequisites all set, follow the lab instructions sequentially. First, clone the github repository consisting of a simple RViz model, a gazebo world, launch files, and a simple script in both Python and C++ . If you encounter any issues throughout the trial, you can direct to the debugging issues file provided.

Go ahead and clone the following github repository:
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
