#!/usr/bin/env bash

rosparam set use_sim_time true
rosbag play --clock $@
