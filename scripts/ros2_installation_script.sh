set -e  # Exit immediately if a command exits with a non-zero status
export DEBIAN_FRONTEND=noninteractive

# Function to display messages in green
echo_message() {
    echo -e "\n\033[1;32m$1\033[0m\n"
}

# 1. Set Up Locale
echo_message "Setting up locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify locale settings
locale

# Enable Required Repositories
echo_message "Enabling required repositories..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG Key and Repository
echo_message "Adding ROS 2 GPG key and repository..."
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
# The following command uses the Ubuntu codename from /etc/os-release.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add Zenoh deps
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list > /dev/null

# Install Development libs
echo_message "Installing development tools..."
sudo apt update && sudo apt install -y \
    python3-pip protobuf-compiler libprotobuf-dev \
    python3-colcon-common-extensions libzenohcpp-dev libzenohc-dev \
    libcurl4-openssl-dev clang-tidy clang-format flex bison gperf wget curl \
    usbutils python3-vcstool

# Install ROS 2 Jazzy
echo_message "Installing ROS 2 Jazzy..."
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop zenoh libprotobuf-dev libzenohcpp-dev libzenohc-dev python3-rosdep

# Set Up Environment
echo_message "Setting up environment..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

# Initialize rosdep
sudo rosdep init

echo_message "ROS 2 Jazzy installation and verification complete!"
