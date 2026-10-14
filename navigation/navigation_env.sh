#!/bin/bash
# Source after AidLux ROS and lidar_ws; vendor libraries always take precedence.
NAV_PREFIX="$HOME/robot/deps/nav2/opt/ros/humble"
export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:+$AMENT_PREFIX_PATH:}$NAV_PREFIX"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$NAV_PREFIX/lib:$NAV_PREFIX/lib/aarch64-linux-gnu"
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$NAV_PREFIX/local/lib/python3.10/dist-packages:$NAV_PREFIX/lib/python3.10/site-packages"
export PATH="$PATH:$NAV_PREFIX/bin"
unset NAV_PREFIX
