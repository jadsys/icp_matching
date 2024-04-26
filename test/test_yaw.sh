#!/bin/bash

echo "First arg: $1"
echo "Second arg: $2"

export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311

# Creating test case folders
mkdir /home/ros/work/20230613_地図生成/originYaw_$1

# Map distribution and rosbag recording
roslaunch icp_matching test_yaw.launch dst_bag:=/home/ros/work/20230613_地図生成/originYaw_$1/originYaw_$1_out.bag &

# Wait for ipc completion
sleep 5

# kill map server
rosnode kill /map_server_target

# kill rosbag
rosnode kill /bag_saver

# Take a screenshot of the entire screen
gnome-screenshot -f /home/ros/work/20230613_地図生成/originYaw_$1/originYaw_$1.png