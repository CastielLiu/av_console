#!/bin/sh

#roslaunch recorder record_drive_status.launch file_path:=$1
roslaunch recorder record_gps_ins_node.launch file_path:=$1

