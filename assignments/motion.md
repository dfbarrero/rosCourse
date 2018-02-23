# Programatic robot motion

## Objectives

1.- First contact with ROS programming model.

2.- Deal with some of the main problems involved in robot control.

## Prerequisites

Install the following ROS packages:

* stdr_simulator
* stdr_resources
* stdr_gui
* stdr_robot

Download the [launch file](simple_room.launch), which contains the scenario for this assignment. Verify that you can correctly run the launch file with roslaunch.

## Practical assignment

The goal to achieve in this assignmen is, at a first look, easy. Given the robot placed in the map shown below, move it from its initial location on point A to the point B.

![Map](simple_rooms.png)

To this end, perform the following tasks:

1.- Create a new ROS package named simple_controller.

2.- Create a subfolder launch.

3.- Move the simple_room.launch file to the launch folder.

4.- Implement a Python node which moves the robot from A to B.

This is not a task as easy as it seems. You will have to deal with several difficulties, please, ask the instructor as those difficulties arise.
