#!/bin/bash
set -e

PROJECT_ROOT="$(realpath "$(dirname "$0")/..")"

# Configuration – update these variables as needed
REMOTE_USER="${REMOTE_USER:-demo}"                      # Your Raspberry Pi username
REMOTE_HOST="${REMOTE_HOST:-u-opentier-pi4.local}"        # Replace with your Raspberry Pi IP or hostname
BUNDLE_NAME=project_bundle.tar.gz
LOCAL_BUNDLE="${PROJECT_ROOT}/${BUNDLE_NAME}"            # Path to the local bundle archive
REMOTE_BUNDLE="/home/${REMOTE_USER}/${BUNDLE_NAME}"      # Remote path where the bundle will be copied
REMOTE_APP_DIR="/home/${REMOTE_USER}/app"                # Remote directory where the bundle will be extracted

# Ensure SSH key-based authentication is set up to avoid password prompts
if ! ssh -o BatchMode=yes "${REMOTE_USER}@${REMOTE_HOST}" 'exit' 2>/dev/null; then
    echo "SSH key-based authentication is not set up. Setting it up now..."
    ssh-copy-id "${REMOTE_USER}@${REMOTE_HOST}"
fi

echo "Copying bundle archive to remote Raspberry Pi..."
scp "$LOCAL_BUNDLE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_BUNDLE}"

# Prompt for remote sudo password if not provided
if [ -z "$REMOTE_PASSWORD" ]; then
    read -sp "Enter remote sudo password for ${REMOTE_USER}@${REMOTE_HOST}: " REMOTE_PASSWORD
    echo ""
fi

# Stop the service if it's already running
echo "Stopping currently running edge_device_launch service (if any)..."
ssh -t "${REMOTE_USER}@${REMOTE_HOST}" "echo '$REMOTE_PASSWORD' | sudo -S systemctl stop edge_device_launch.service || true"

echo "Removing old app directory on remote..."
ssh "${REMOTE_USER}@${REMOTE_HOST}" "rm -rf '$REMOTE_APP_DIR'"

echo "Creating new app directory on remote..."
ssh "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p '$REMOTE_APP_DIR'"

echo "Extracting bundle archive on remote into $REMOTE_APP_DIR..."
ssh "${REMOTE_USER}@${REMOTE_HOST}" "tar -xzf '$REMOTE_BUNDLE' -C '$REMOTE_APP_DIR'"

echo "Removing bundle archive from remote..."
ssh "${REMOTE_USER}@${REMOTE_HOST}" "rm -f '$REMOTE_BUNDLE'"

###############################################################################
# Create the wrapper script on the remote machine and install it to /usr/bin
###############################################################################
echo "Creating wrapper script on remote..."
cat <<EOF | ssh "${REMOTE_USER}@${REMOTE_HOST}" "tee /tmp/edge_device_launch_wrapper.sh > /dev/null"
#!/bin/bash
# Wrapper script to set up the ROS2 environment and launch the node.
# Source the ROS2 environment from the deployed bundle.
export ROS_DOMAIN_ID=42
source ${REMOTE_APP_DIR}/install/setup.bash
ros2 launch edge_device_launch edge_device_launch.py
EOF

# Move the temporary wrapper script to /usr/bin and make it executable.
ssh -t "${REMOTE_USER}@${REMOTE_HOST}" "echo '$REMOTE_PASSWORD' | sudo -S mv /tmp/edge_device_launch_wrapper.sh /usr/bin/edge_device_launch_wrapper.sh"
ssh -t "${REMOTE_USER}@${REMOTE_HOST}" "echo '$REMOTE_PASSWORD' | sudo -S chmod +x /usr/bin/edge_device_launch_wrapper.sh"

###############################################################################
# Create the systemd service file that uses the wrapper script.
###############################################################################
echo "Creating/updating systemd service for edge_device_launch..."
cat <<EOF | ssh "${REMOTE_USER}@${REMOTE_HOST}" "tee /tmp/edge_device_launch.service > /dev/null"
[Unit]
Description=Edge Device Launch Service
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=${REMOTE_USER}
WorkingDirectory=${REMOTE_APP_DIR}
# Ensure that the PATH includes the ROS2 installation directory.
Environment="PATH=/opt/ros/foxy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
ExecStart=/bin/bash /usr/bin/edge_device_launch_wrapper.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

# Move the temporary service file into place with sudo.
ssh -t "${REMOTE_USER}@${REMOTE_HOST}" "echo '$REMOTE_PASSWORD' | sudo -S mv /tmp/edge_device_launch.service /etc/systemd/system/edge_device_launch.service"

# Reload systemd, enable, and start the service.
echo "Reloading systemd daemon..."
ssh -t "${REMOTE_USER}@${REMOTE_HOST}" "echo '$REMOTE_PASSWORD' | sudo -S systemctl daemon-reload"
echo "Enabling edge_device_launch service to start on boot..."
ssh -t "${REMOTE_USER}@${REMOTE_HOST}" "echo '$REMOTE_PASSWORD' | sudo -S systemctl enable edge_device_launch.service"
echo "Starting edge_device_launch service..."
ssh -t "${REMOTE_USER}@${REMOTE_HOST}" "echo '$REMOTE_PASSWORD' | sudo -S systemctl start edge_device_launch.service"

echo "Deployment complete. The bundle has been extracted to $REMOTE_APP_DIR on $REMOTE_HOST and the service is running."
