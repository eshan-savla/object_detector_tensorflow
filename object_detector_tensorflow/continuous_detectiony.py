#!/usr/bin/env python3
import argparse

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
import cv2
import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow.srv import DetectObjects
from object_detector_tensorflow.msg import Detection


class ContinuousDetectionNode(ObjectDetectionBaseNode):

    def __init__(self,
                 saved_model_path,
                 label_map_path,
                 min_probability=0.5,
                 max_gpu_memory_fraction=None,
                 result_image_size=(640, 480),
                 log_level=rclpy.logging.LoggingSeverity.INFO):

        super().__init__(saved_model_path,
                         label_map_path,
                         min_probability,
                         max_gpu_memory_fraction,
                         result_image_size,
                         log_level)

        self.service = self.create_service(
            DetectObjects, "DetectObjects", self._detect_objects)

    def _detect_objects(self, request, response):

        detected_objects, image = super()._detect_objects(
            request.image, request.roi)

        # for detection in detected_objects:
        #     detection.header.stamp = self.get_clock().now().to_msg()
        #     detection.header.frame_id = "object_detection"

        response.detections = detected_objects
        response.image = image

        return response


def main(args=None):

    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        description='Object Detection TensorFlow ROS node')
    parser.add_argument('-n', '--node_name', default="object_detector_tensorflow",
                        help="ROS node name")
    parser.add_argument('--image_topic', default="/stereo/left/image_rect_color",
                        help="ROS topic to listen for images")
    parser.add_argument('--saved_model_path', default="/saved_model",
                        help="Path to TensorFlow saved model folder")
    parser.add_argument('--label_map_path', default="/label_map.txt",
                        help="Text file with class names (one label per line)")
    parser.add_argument('--min_probability', type=float, default=0.99,
                        help="Minimum probability for detections to be reported")
    parser.add_argument('--data_acquisition_timeout', type=float, default=5,
                        help="Data acquisition timeout in seconds")
    parser.add_argument('--max_gpu_memory_fraction', type=float, default=1.0,
                        help="Limits the GPU memory usage of the TensorFlow model to only a fraction (between 0 and 1)")
    parser.add_argument('-v', '--verbose', action="store_true",
                        help="Verbose logging (debug level)")
    args = parser.parse_args()

    ros_log_level = rclpy.logging.LoggingSeverity.INFO

    if args.verbose:
        ros_log_level = rclpy.logging.LoggingSeverity.DEBUG

    result_topic = f"{args.node_name}/detections"

    # may raise PackageNotFoundError
    package_share_directory = get_package_share_directory(
        "object_detector_tensorflow")
    saved_model_path = os.path.join(
        package_share_directory, "..", "..", "lib/object_detector_tensorflow/data/saved_model")
    label_map_path = os.path.join(
        package_share_directory, "..", "..", "lib/object_detector_tensorflow/data/label_map.txt")

    ObjectDetectionNode(saved_model_path,  # =args.saved_model_path,
                        label_map_path,  # =args.label_map_path,
                        image_topic=args.image_topic,
                        result_topic=result_topic,
                        min_probability=args.min_probability,
                        name=args.node_name,
                        data_timeout=args.data_acquisition_timeout,
                        max_gpu_memory_fraction=args.max_gpu_memory_fraction,
                        log_level=ros_log_level).run()


if __name__ == '__main__':
    main()
