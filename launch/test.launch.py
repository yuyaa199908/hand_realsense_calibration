import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hand_realsense_calibration'),
        'config',
        'config.yaml'
    )
    return LaunchDescription([
        Node(
            package='hand_realsense_calibration',
            namespace='data_saver_node',
            executable='data_saver_node',
            remappings=[('/input_img', '/camera/color/image_raw'),],
            parameters=[config]
        ),
    ])
