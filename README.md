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

- Go ahead and clone the following github repository:
  ```
  cd ~
  git clone https://github.com/khouka/RoboticArmPush_Lab.git
  ```
- Note: Don’t forget to source your ROS system whenever opening a new shell
  ```
  $ source /opt/ros/melodic/setup.bash
  ```
- Before you run the example, first compile your code in your workspace and source the development folder of the workspace:
  ```
  $ cd  RoboticArmPush_Lab
  $ catkin_make 
  $ source /home/<account name>/RoboticArmPush_Lab/devel/setup.bash
  ```
- After you have successfully compiled the code without errors, it is time to run the launch file. Inside the workspace, run the following command:
  ```
  $ roslaunch minimal_demo coin_rviz.launch
  ```
- The format of launching a ROS launch file is the roslaunch command then the <package name>, and lastly the <launch file>. You should see 2 popup displays:
   IMAGE NUMBER 1
- Now that we have RViz running let’s display the robot, to do so, you must adjust a few settings in the RViz: 
  - Under the displays panel, there is a fixed Frame, click on the word map, select base_link.
     IMAGE NUMBER 2  
  - There should be an Add bottom, at the bottom part of the RViz, click on the Add ,  scroll down and select the RobotModel. 
  - After adding the RobotModel the robot should appear, you can move the robot using the joint_state_publisher pop up. As you can see by moving joint 1 we are rotating around the z axis, and on joint 2 we are rotating around the y axis. 
   ![file:///Users/amarkhoukhi/Desktop/f.png]
- Let’s configure and save the progress, so that you won’t have to readjust the settings every time you run the simulation. 
  - After adjusting the settings. Click on File in the top left corner, then Save Config As:
  IMAGE 4
  - Save the configuration inside the launch directory and name it coin_config.rviz
  - Lastly, make an adjustment in the coin_rviz.launch file by adding: 
    ```
    args=" -d $(find demo)/launch/coin_config.rviz"
    ```
    after the type="rviz" part of the code. Save and close the file. 
- Now if you launch the file again, the robot model should appear, as the adjustments were saved. Now let’s break down the key code segments! Now let’s go over some basics of ROS, then we will take a look at the code you just ran. 

## ROS Basics:
In this segment, we will briefly go over some of the basics of using ROS. If you want to further learn about ROS, you can also head to the following link: (http://wiki.ros.org/ROS/Tutorials). 
  1. Workspace:
     A catkin workspace is a folder where you modify, build, and install catkin packages. All your work for a specific   project should be encompassed in a workspace. To create a workspace type:
     ```
     $ mkdir <workspace name>
     ```
     It is also good practice to keep all your packages inside an src folder in the workspace:
     ```
     $ cd <workspace name>
     $ mkdir src
     ```
  2. Package:
     A package is a directory that contains all of the files, programs, libraries, and datasets needed to provide some   useful functionality. Every program that you write in ROS will need to be inside a package. To create a package:
     ```
     $ catkin_create_pkg <package name>
     ```

  3. Adding dependencies:
     Inside your catkin package you will see two files: CMakeLists.txt and package.xml.
     - CMakeLists.txt:
       CMakeLists.txt file contains a set of directives and instructions describing the project's  source files and          targets. To add depencies, search for the lines:
       IMAGE
       Edit to:
       IMAGE
     - Package.xml:
       The same dependencies you added to your CMakelist.txt, you must add here. 
       IMAGE
       Add the dependencies in this format, make you sure you save the file afterwards.
       ```
       <build_depend>package name</build_depend>
       <exec_depend>package name</exec_depend>
       ```
  4. Compiling and sourcing:
     After editing the workspace, you must compile your code using:
     ```
     $ catkin_make
     ```
     After compiling you must source the development folder of workspace:
     ```
     $ source /home/<account name>/<workspace name>/devel/setup.bash
     ```
## The Code Explained
Inside `Example/` folder, there're three basic example scripts, showing how to read keyboard inputs, drive a turtle with python turtle module and detect red object.


