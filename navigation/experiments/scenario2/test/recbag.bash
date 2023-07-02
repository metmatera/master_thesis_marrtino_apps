#!/bin/bash

echo Filename:
read filename

rosbag record -O $filename /base_pose_ground_truth /cmd_vel /human1/base_pose_ground_truth /human1/cmd_vel

