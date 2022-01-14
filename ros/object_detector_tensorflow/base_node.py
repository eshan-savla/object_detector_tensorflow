#!/usr/bin/env python3

import os

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

from odtf.object_detection import ObjectDetection
from odtf.visualization import Visualization
from object_detector_tensorflow.diagnostics import Diagnostics
from object_detector_tensorflow.srv import DetectObjects
from object_detector_tensorflow.msg import Detection, Detections


class ObjectDetectionBaseNode(Node):

    def __init__(self, node_name):

        super().__init__(node_name)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('saved_model_path', "data/saved_model"),
                ('label_map_path', "data/label_map.txt"),
                ('image_topic', "/image"),
                ('min_probability', 0.5),
                ('max_gpu_memory_fraction', 1.0),
                ('result_image_size', [640, 480])
            ])

        self.saved_model_path = os.path.abspath(os.path.expanduser(
            self.get_parameter("saved_model_path").get_parameter_value().string_value))
        self.label_map_path = os.path.expanduser(
            self.get_parameter("label_map_path").get_parameter_value().string_value)
        self.image_topic = self.get_parameter(
            "image_topic").get_parameter_value().string_value
        self.min_probability = self.get_parameter(
            "min_probability").get_parameter_value().double_value
        self.max_gpu_memory_fraction = self.get_parameter(
            "max_gpu_memory_fraction").get_parameter_value().double_value
        self.result_image_size = tuple(self.get_parameter(
            "result_image_size").get_parameter_value().integer_array_value)

        self.logger = self.get_logger()

        self.detection = ObjectDetection(self.saved_model_path,
                                         self.label_map_path,
                                         self.max_gpu_memory_fraction)

        self.visualization = Visualization()

        self.diagnostics = Diagnostics(node=self)

        self.bridge = CvBridge()

    def run(self):

        try:
            self.logger.info("Started Node")

            rclpy.spin(self)

        except KeyboardInterrupt:
            self.logger.info("Stopped Node")

        self.destroy_node()
        rclpy.shutdown()

    def _detect_objects(self, image: Image, roi: RegionOfInterest = None):

        self.logger.info("Detecting objects")

        detected_objects = None
        result_image = None

        if roi is not None:
            if not any([roi.x_offset, roi.y_offset,
                        roi.width, roi.height]):
                roi = None

        try:
            image_header = image.header
            image = self.bridge.imgmsg_to_cv2(image)

            # if grayscale image, stack it to comply to
            # the 3 different channels of input tensor
            if len(image.shape) == 2:
                image = np.stack((image,)*3, axis=-1)

            start_time = self.get_clock().now()

            box = None

            if roi is not None:
                box = [roi.y_offset,
                       roi.x_offset,
                       roi.y_offset + roi.height,
                       roi.x_offset + roi.width]

            detections = self._clean_detections(self.detection.run(image, box))

            # self.diagnostics.update(
            #     "last_timestamp_processed", image_header.stamp.to_sec())
            # self.diagnostics.update(
            #     "detection_time", self.get_clock().now() - start_time)

            self.logger.info("Detected {} objects".format(
                len(detections)))

            self.logger.debug(f"{detections}")

            detected_objects = self._convert_detections_to_ros(
                detections, image_header)

            result_image = self._convert_image_to_ros(
                self.visualization.draw_detections(
                    image, detections, box))

        except CvBridgeError as e:
            self.logger.error(str(e))

        except Exception as e:
            self.logger.error(f"Caught {type(e)} during detection: {e}")

        return detected_objects, result_image

    def _clean_detections(self, raw_detections):

        detections = []

        for detection in raw_detections:
            if detection["probability"] >= self.min_probability:
                detections.append(detection)

        return detections

    def _convert_detections_to_ros(self, detections, image_header):

        detected_objects = Detections()

        for detection in detections:
            bounding_box = RegionOfInterest(
                y_offset=detection["bounding_box"][0],
                x_offset=detection["bounding_box"][1],
                height=(detection["bounding_box"][2] -
                        detection["bounding_box"][0]),
                width=(detection["bounding_box"][3] -
                       detection["bounding_box"][1]))

            detected_objects.detections.append(Detection(
                class_id=detection["class_id"],
                class_name=detection["class_name"],
                probability=detection["probability"],
                bounding_box=bounding_box))

        detected_objects.image_header = image_header
        detected_objects.header.stamp = self.get_clock().now().to_msg()
        detected_objects.header.frame_id = "object_detector_tensorflow"

        return detected_objects

    def _convert_image_to_ros(self, image: np.ndarray):
        try:
            image = cv2.resize(image,
                               dsize=self.result_image_size,
                               interpolation=cv2.INTER_AREA)

            image = self.bridge.cv2_to_imgmsg(
                image, encoding="bgr8")

        except CvBridgeError as e:
            self.logger.error(e)

        return image
