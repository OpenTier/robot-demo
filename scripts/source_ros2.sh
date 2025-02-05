#!/bin/bash

# Make sure system Python is used
export PYTHON_EXECUTABLE=/usr/bin/python3

# Remove the ESP32 python from the PATH if it is at the front
export PATH=$(echo $PATH | sed -e 's|/home/dev/.espressif/python_env/idf5.5_py3.12_env/bin:||')

# Source the ROS 2 environment
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
