#!/bin/bash
set -e

PROJECT_ROOT="$(realpath "$(dirname "$0")/..")"

# -- Configuration --
REMOTE_USER="${REMOTE_USER:-demo}"
REMOTE_HOST="${REMOTE_HOST:-u-opentier-pi4.local}"
REMOTE_PROJECT_DIR="/home/${REMOTE_USER}/ros2_project"

# Ensure SSH key-based authentication is set up to avoid password prompts
ensure_ssh_is_setup() {
    if ! ssh -o BatchMode=yes "${REMOTE_USER}@${REMOTE_HOST}" 'exit' 2>/dev/null; then
        echo "SSH key-based authentication is not set up. Setting it up now..."
        ssh-copy-id "${REMOTE_USER}@${REMOTE_HOST}"
    fi
}

# Create the remote project directory
prepare_remote_dir() {
    echo "Creating remote project directory..."
    ssh "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p '${REMOTE_PROJECT_DIR}'"
}

# Remove the remote project directory if it exists
clean_old_dir() {
    echo "Cleaning the remote project directory if it exists..."
    ssh "${REMOTE_USER}@${REMOTE_HOST}" "if [ -d '${REMOTE_PROJECT_DIR}' ]; then rm -rf '${REMOTE_PROJECT_DIR}'; fi"
}

# Copy the project directories to the Raspberry Pi
copy_project() {
    echo "Copying project code to the Raspberry Pi..."
    prepare_remote_dir
    scp -r "${PROJECT_ROOT}/edge_device" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PROJECT_DIR}"
    scp -r "${PROJECT_ROOT}/micro_ros_setup" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PROJECT_DIR}"
    scp -r "${PROJECT_ROOT}/vehicle-cloud-api" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PROJECT_DIR}"
}

# Setup uROS on the Raspberry Pi by running commands remotely via SSH
setup_uros() {
    echo "Setting up uROS on the Raspberry Pi..."
    ssh "${REMOTE_USER}@${REMOTE_HOST}" << EOF
REMOTE_PROJECT_DIR="${REMOTE_PROJECT_DIR}"
mkdir -p "\${REMOTE_PROJECT_DIR}/uros_ws/src"
if [ -L "\${REMOTE_PROJECT_DIR}/uros_ws/src/micro_ros_setup" ]; then
    echo "Symlink \${REMOTE_PROJECT_DIR}/uros_ws/src/micro_ros_setup already exists."
else
    ln -s "\${REMOTE_PROJECT_DIR}/micro_ros_setup/" "\${REMOTE_PROJECT_DIR}/uros_ws/src/micro_ros_setup"
    echo "Symlink created."
fi
cd "\${REMOTE_PROJECT_DIR}/uros_ws/"
rosdep update
sudo rosdep install --from-paths src --ignore-src -y
if [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
else
    echo "Error: /opt/ros/jazzy/setup.bash not found."
    exit 1
fi
colcon build
if [ -f ./install/setup.bash ]; then
    . ./install/setup.bash
else
    echo "Error: Workspace setup script (./install/setup.bash) not found."
    exit 1
fi
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
EOF
}

# Execute the functions in order
ensure_ssh_is_setup

# Check if the first argument is "clean"
if [ "$1" == "clean" ]; then
    clean_old_dir
fi

copy_project
setup_uros
