#!/bin/bash
set -e

PROJECT_ROOT="$(realpath "$(dirname "$0")/..")"
# Configuration: update these to match your Raspberry Pi's details.
INSTALLATION_SCRIPT_PATH="${PROJECT_ROOT}/scripts/ros2_installation_script.sh"
REMOTE_USER="${REMOTE_USER:-demo}"                       # Raspberry Pi username
REMOTE_HOST="${REMOTE_HOST:-u-opentier-pi4.local}"       # Raspberry Pi IP or hostname

# Ensure SSH key-based authentication is set up to avoid password prompts
if ! ssh -o BatchMode=yes "${REMOTE_USER}@${REMOTE_HOST}" 'exit' 2>/dev/null; then
    echo "SSH key-based authentication is not set up. Setting it up now..."
    ssh-copy-id "${REMOTE_USER}@${REMOTE_HOST}"
fi

# Copy the installation script to the Raspberry Pi's /tmp directory
echo "Copying installation script to ${REMOTE_HOST}..."
scp $INSTALLATION_SCRIPT_PATH "${REMOTE_USER}@${REMOTE_HOST}:/tmp/ros2_installation_script.sh"

# Execute the installation script remotely as root
# The -t flag allocates a pseudo-tty, ensuring sudo can prompt for a password once if needed.
echo "Executing installation script on ${REMOTE_HOST}..."
ssh -t "${REMOTE_USER}@${REMOTE_HOST}" "sudo bash /tmp/ros2_installation_script.sh"

# Remove the installation script from the remote machine
echo "Cleaning up installation script on ${REMOTE_HOST}..."
ssh "${REMOTE_USER}@${REMOTE_HOST}" "sudo rm /tmp/ros2_installation_script.sh"

echo "ROS 2 installation complete!"
