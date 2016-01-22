TeraRanger Tower ROS Module
=========================

This is the ROS module for the TeraRanger Tower ranging sensor (www.teraranger.com).


Using module
============

To use the ROS node you just need to:
* Create a ROS Workspace
* Copy the node teraranger_tower package into the workspace src directory
* Compile using: catkin_make 
* Setup environment: source devel/setup.sh
* Run using: rosrun teraranger_tower teraranger_tower_node _portname:=/dev/ttyACM0

If you want to change the operating mode, run
* rosrun rqt_reconfigure rqt_reconfigure 

NB: remember to execute the daemon roscore before running the rosrun command
