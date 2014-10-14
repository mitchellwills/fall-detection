#!/usr/bin/env bash

rosbag record -b 0 /camera/rgb/camera_info /camera/rgb/image_color /tf /tf_static
