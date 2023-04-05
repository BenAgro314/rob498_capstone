#!/usr/bin/bash

#export DISPLAY=:1
export GROUNDSTATION_IP=10.42.0.131 #100.66.93.210 # 10.42.0.131 --- set this if using myhal wifi
#export ROS_MASTER_URI=http://$GROUNDSTATION_IP:11311
#export ROS_IP=10.42.0.101
export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP

