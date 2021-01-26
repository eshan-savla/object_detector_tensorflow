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

from object_detector_tensorflow.scripts.object_detection import ObjectDetection
from object_detector_tensorflow.scripts.visualization import Visualization
from object_detector_tensorflow.scripts.diagnostics import Diagnostics
from object_detector_tensorflow.srv import DetectObjects
from object_detector_tensorflow.msg import Detection


class ObjectDetectionBaseNode(Node):

    def __init__(self,
                 saved_model_path,
                 label_map_path,
                 min_probability=0.5,
                 data_timeout=1.5,
                 max_gpu_memory_fraction=None,
                 result_image_size=(640, 480),
                 log_level=rclpy.logging.LoggingSeverity.INFO):

        super().__init__("object_detector_tensorflow")

        self.logger = self.get_logger()

        self.collection_timeout = Duration(seconds=data_timeout)
        self.min_probability = min_probability
        self.result_image_size = result_image_size

        self.detection = ObjectDetection(saved_model_path,
                                         label_map_path,
                                         max_gpu_memory_fraction)

        self.visualization = Visualization()

        self.diagnostics = Diagnostics(node=self)

        self.publisher = self.create_publisher(
            Image, f"{self.get_name()}/detections", 1)

        self.bridge = CvBridge()

        self.image_msg = None

    def run(self):

        try:
            self.logger.info("Started Node")

            rclpy.spin(self)

        except KeyboardInterrupt:
            self.logger.info("Stopped Node")

        self.destroy_node()
        rclpy.shutdown()

    def _publish_image(self, image: np.ndarray):

        try:
            image = cv2.resize(image,
                               dsize=self.result_image_size,
                               interpolation=cv2.INTER_AREA)

            self.publisher.publish(self.bridge.cv2_to_imgmsg(
                image, encoding="bgr8"))

        except CvBridgeError as e:
            self.logger.error(e)

    def _detect_objects(self, image: Image, roi: RegionOfInterest = None):

        self.logger.info("Detecting objects")

        detected_objects = None

        if roi is not None:
            if not any([roi.x_offset, roi.y_offset,
                        roi.width, roi.height]):
                roi = None

        try:
            image = self.bridge.imgmsg_to_cv2(image)
            image_header = image.header

            # if grayscale image, stack it to comply to
            # the 3 different channels of input tensor
            if len(image.shape) == 2:
                image = np.stack((image,)*3, axis=-1)

            start_time = self.get_clock().now()

            box = [roi.y_offset,
                   roi.x_offset,
                   roi.y_offset + roi.height,
                   roi.x_offset + roi.width]

            detected_objects = self._convert_to_ros(
                self.detection.run(image, box))

            visualization_image = self.visualization.draw_detections(
                image, detected_objects, box)

            # self.diagnostics.update(
            #     "last_timestamp_processed", image_header.stamp.to_sec())
            # self.diagnostics.update(
            #     "detection_time", self.get_clock().now() - start_time)

            self._publish_image(visualization_image)

            self.logger.info("Detected {} objects".format(
                len(detected_objects)))

            self.logger.debug(f"{detected_objects}")

        except CvBridgeError as e:
            self.logger.error(e)

        except Exception as e:
            self.logger.error(f"Caught {type(e)} during detection: {e}")

        return detected_objects, visualization_image

    def _convert_to_ros(self, raw_detections):

        detections = []

        for detection in raw_detections:
            if detection["probability"] >= self.min_probability:
                bounding_box = RegionOfInterest(
                    y_offset=detection["bounding_box"][0],
                    x_offset=detection["bounding_box"][1],
                    height=(detection["bounding_box"][2] -
                            detection["bounding_box"][0]),
                    width=(detection["bounding_box"][3] -
                           detection["bounding_box"][1]))

                detections.append(Detection(
                    class_id=detection["class_id"],
                    class_name=detection["class_name"],
                    probability=detection["probability"],
                    bounding_box=bounding_box))

        return detections
