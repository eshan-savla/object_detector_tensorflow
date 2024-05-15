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
            executable='detect_and_transform_node',
            name='detect_and_transform_node',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='point_transformation',
            executable='point_transformation_node',
            name='point_transformation_node',
            output='screen',
            parameters=[config]
        )
        # Node(
        #     package='object_detector_tensorflow',
        #     executable='client_detect_and_transform',
        #     name='client_detect_and_transform',
        #     output='screen',
        #     parameters=[config]
        # )
    ])
