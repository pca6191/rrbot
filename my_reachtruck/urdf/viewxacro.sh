#!/bin/bash

# convert xacro to urdf
rosrun xacro xacro $1 > /tmp/$1.urdf

# show urdf in rviz
roslaunch urdf_tutorial display.launch model:=/tmp/$1.urdf
