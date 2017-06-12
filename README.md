msl_ros_common
===============

Author: Eric Cristofalo

Affiliation: Stanford University

Date Created: 2017/06/08; Date Last Modified: 2017/06/08

Tested on: ROS Kinetic

# mocap_optitrack
Standard ROS package for publishing data from Optitrack:
http://wiki.ros.org/mocap_optitrack

## Modified Files:
* mocap_datapackets.cpp
	* Source file that reads raw optitrack data. We have modified this file to make initial coordinate frame transformations that work with our system. 
* mocap.yaml
	* Configuration file that defines the rigib body topics to be defined by mocap_otitrack. If you are using mutiple rigid bodies, uncomment sections of this configuration file

# mocap_interface
This package contains an interfacing node and launch file for the mocap_optitrack package. The idea is to use this package to communicate transform the original data from mocap_optitrack into the coordinate frames used in lab. 

## Requirements

## Files
* mocap_interface.cpp
	* Reads raw Optitrack data and outputs rigid body pose in geometry_msgs/Twist 
    * COORDINATE_FRAME_INDEX: coordinate frame index (0 for native mocap_optitrack output)
    * DISPLAY_DATA_BOOL: integer indicator for printing data during testing (1 displays data)
* mocap_interface_odom.cpp
	* Reads raw Optitrack data and outputs a nav_msgs/Odometry message for SLAM packages 
    * DISPLAY_DATA_BOOL: integer indicator for printing data during testing (1 displays data)
    * POSITION_COVARIANCE: double for position covariance
    * ORIENTATION_COVARIANCE: double for orientation covariance
    * VELOCITY_COVARIANCE: double for velocity covariance
    * ROTATION_RATE_COVARIANCE: double for rotation rate covariance
* mocap.yaml
	* Configuration file that defines the rigib body topics to be defined by mocap_otitrack. If you are using mutiple rigid bodies, uncomment sections of this configuration file
* mocap.launch
    * Generic setup launch file that contains all available ROS parameters. Feel free to copy and simplify this file for your project's launch file. 

