# Use ROS 2 Jazzy as the base image
FROM ros:jazzy

# Define user arguments
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ARG PROJECT_DIR=/home/$USERNAME/opentier-robot-demo

# Create the non-root user with sudo privileges
RUN if id -u $USER_UID > /dev/null 2>&1; then sudo userdel $(id -un $USER_UID); fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    apt-get update && apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -a -G dialout $USERNAME

# Add Zenoh repository
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null

# Install dependencies required for ROS 2 and ESP32 development
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    python3-pip python3-venv python-is-python3 \
    git wget curl libcurl4-openssl-dev clang-tidy clang-format usbutils \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util \
    screen minicom picocom flex bison gperf \
    libusb-1.0-0 openssh-client bash-completion libnss-mdns \
    protobuf-compiler libprotobuf-dev \
    zenoh libzenohcpp-dev libzenohc-dev && \
    # ROS 2 visualization tools
    # ros-jazzy-rqt-graph ros-jazzy-rqt-common-plugins && \
    rm -rf /var/lib/apt/lists/*

# --- ESP-IDF (ESP32) Setup ---
# Clone ESP-IDF repository as root
WORKDIR /opt
RUN git clone --recursive https://github.com/espressif/esp-idf.git esp-idf && \
    chown -R $USERNAME:$USERNAME /opt/esp-idf

# Switch to non-root user to run the ESP-IDF install script.
USER $USERNAME
WORKDIR /opt/esp-idf
RUN ./install.sh esp32

USER root
WORKDIR /opt
RUN git clone --recursive https://github.com/espressif/arduino-esp32.git arduino-esp32 && \
    chown -R $USERNAME:$USERNAME arduino-esp32

USER $USERNAME
WORKDIR /opt/arduino-esp32
RUN git submodule update --init --recursive && \
    bash -c "python3 tools/get.py"

# Install PlatformIO (for ESP32 development) as the non-root user.
RUN python3 -m pip install platformio --break-system-packages
# --- End of ESP32 Setup ---

# Copy the Git branch showing and coloring script and apply it content to the bashrc
COPY show_git_branch.sh /home/$USERNAME/show_git_branch.sh
RUN cat /home/$USERNAME/show_git_branch.sh >> /home/$USERNAME/.bashrc && \
    rm /home/$USERNAME/show_git_branch.sh

# Set the working directory to the user's workspace.
WORKDIR /home/$USERNAME/workspace

# Switch to non-root.
USER $USERNAME
ENV HOME=/home/$USERNAME
ENV SHELL=/bin/bash

# Start with bash.
CMD ["/bin/bash"]
