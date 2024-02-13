import os

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
        'object_detector_tensorflow',
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_detector_tensorflow',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[config]
        )
    ])
