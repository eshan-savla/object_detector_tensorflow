import os
from glob import glob
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_prefix('object_detector_tensorflow'),
        '..',
        '..',
        'src',
        'object_detector_tensorflow',
        'ros',
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_detector_tensorflow',
            node_executable='detection_node.py',
            node_name='detection_node',
            output='screen',
            parameters=[config]
        )
    ])
