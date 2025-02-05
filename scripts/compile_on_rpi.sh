#!/bin/bash
set -e

# -- Configuration --
REMOTE_USER="${REMOTE_USER:-demo}"
REMOTE_HOST="${REMOTE_HOST:-u-opentier-pi4.local}"
REMOTE_PROJECT_DIR="/home/${REMOTE_USER}/ros2_project"
# (Unused variables below have been left here in case you need them later)
PROJECT_BUNDLE_NAME="project_bundle.tar.gz"
REMOTE_BUILD_ARCHIVE="/home/${REMOTE_USER}/${PROJECT_BUNDLE_NAME}"

ensure_ssh_is_setup() {
    if ! ssh -o BatchMode=yes "${REMOTE_USER}@${REMOTE_HOST}" 'exit' 2>/dev/null; then
        echo "SSH key-based authentication is not set up. Setting it up now..."
        ssh-copy-id "${REMOTE_USER}@${REMOTE_HOST}"
    fi
}

compile_project() {
    echo "Compiling the project on the Raspberry Pi..."
    ssh "${REMOTE_USER}@${REMOTE_HOST}" << EOF
source /opt/ros/jazzy/setup.bash
source ${REMOTE_PROJECT_DIR}/uros_ws/install/setup.bash
cd ${REMOTE_PROJECT_DIR}/
echo "Removing existing install directory..."
rm -rf ${REMOTE_PROJECT_DIR}/install
echo "Building the project..."
colcon build --merge-install
echo "Create a tarball of the install directory"
tar -czf edge_device_bundle.tar.gz install
# Move the tarball to the install REMOTE_BUILD_ARCHIVE directory
mv edge_device_bundle.tar.gz ${REMOTE_BUILD_ARCHIVE}
echo "Project compiled successfully."
EOF
}

download_project_bundle() {
    echo "Downloading the project bundle from the Raspberry Pi..."
    scp "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_BUILD_ARCHIVE}" .
    echo "Project bundle downloaded successfully."
    # remove the project bundle from the Raspberry Pi
    ssh "${REMOTE_USER}@${REMOTE_HOST}" "rm ${REMOTE_BUILD_ARCHIVE}"
}

ensure_ssh_is_setup
compile_project
download_project_bundle