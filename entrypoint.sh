#!/usr/bin/env bash

source install/setup.bash
ros2 launch turtlebot2_nav nav2.launch.py namespace:=tb2_5
