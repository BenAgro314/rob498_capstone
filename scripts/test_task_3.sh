#!/bin/bash

rostopic pub rob498_drone_01/comm/waypoints geometry_msgs/PoseArray -f src/offboard_py/config/pose_list_ct4.yaml
