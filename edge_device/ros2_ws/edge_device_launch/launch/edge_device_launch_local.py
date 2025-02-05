import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the package share directory where your config file is installed
    package_share_dir = get_package_share_directory('edge_device_launch')
    config_file = os.path.join(package_share_dir, 'config', 'zenoh_config.json5')

    return LaunchDescription([
        # Start the micro-ROS Agent
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888', '-v4'],
            output='screen'
        ),
        # Launch the cloud_comm_node
        Node(
            package='cloud_comm_node',
            executable='cloud_comm_node',
            name='cloud_comm_node',
        ),
        # Launch the robot_comm_node
        Node(
            package='robot_comm_node',
            executable='robot_comm_node',
            name='robot_comm_node',
        ),
        # Launch the system_status_node
        Node(
            package='system_status_node',
            executable='system_status_node',
            name='system_status_node',
        ),
        # Launch the vision_node
        # Node(
        #     package='vision_node',
        #     executable='vision_node',
        #     name='vision_node',
        # )
    ])