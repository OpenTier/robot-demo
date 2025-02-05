# Edge Device

## Build Edge Device Code

To build the edge device code, run the following commands:

```sh
cd edge_device
. ./scripts/source_ros2.sh
colcon build --merge-install
```

## Run Edge Device Code

In a separate terminal, execute the following:

```sh
cd edge_device
. ./install/setup.bash
ros2 launch edge_device_launch edge_device_launch.py
```

## Scripts

**Important:** Most of these scripts should be run from the project root.

- **build_rpi_bundle.sh**
Used to cross-compile the ROS 2 project for Raspberry Pi. Run it from the host (not within Docker) and from the project root. It will cross-compile the entire ros2_ws for use on the Raspberry Pi, creating a root/bundle_output directory that contains a workspace/ros2_bundle.tar.gz file with the binaries.

- **transfer_rpi_bundle.sh**
Copies the bundle to the Raspberry Pi, removes old applications, and unzips the bundle. Check the script for parameters such as the Raspberry Pi username and host details.

- **source_ros2.sh**
Sources the ROS 2 environment variables.

- **install_ros2_on_rpi.sh**
Copies the ros2_installation_script.sh to the Raspberry Pi and invokes it to install all runtime dependencies.

- **ros2_installation_script.sh**
Installs the ROS 2 Jazzy runtime dependencies.
