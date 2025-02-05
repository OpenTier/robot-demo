#!/bin/bash
set -e

# Create workspace source directory if it doesn't exist
mkdir -p /home/dev/opentier-robot-demo/uros_ws/src

# Create the symlink only if it doesn't already exist
if [ -L "/home/dev/opentier-robot-demo/uros_ws/src/micro_ros_setup" ]; then
    echo "Symlink /home/dev/opentier-robot-demo/uros_ws/src/micro_ros_setup already exists."
else
    ln -s /home/dev/opentier-robot-demo/micro_ros_setup/ /home/dev/opentier-robot-demo/uros_ws/src/micro_ros_setup
    echo "Symlink created."
fi

# Change directory to the workspace root
cd /home/dev/opentier-robot-demo/uros_ws/

# Update rosdep and install dependencies
rosdep update
sudo rosdep install --from-paths src --ignore-src -y

# Source ROS setup file; exit if not found
if [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
else
    echo "Error: /opt/ros/jazzy/setup.bash not found."
    exit 1
fi

# Build the workspace
colcon build

# Source the workspace install setup script; exit if not found
if [ -f ./install/setup.bash ]; then
    . ./install/setup.bash
else
    echo "Error: Workspace setup script (./install/setup.bash) not found."
    exit 1
fi

# Run micro-ROS agent creation and build commands
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
