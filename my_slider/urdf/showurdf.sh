#!/bin/bash
echo "convert file $1 in rviz"
roslaunch urdf_tutorial display.launch model:=$1
