import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('object_detector_tensorflow'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_detector_tensorflow',
            node_executable='detection_node',
            node_name='detection_node',
            output='screen',
            parameters=[config]
        )
    ])
