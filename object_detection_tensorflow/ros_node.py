#!/usr/bin/env python3
import argparse

import tensorflow as tf
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

from object_detection_tensorflow.object_detection import ObjectDetection
from object_detection_tensorflow.visualization import Visualization
from object_detection_tensorflow.diagnostics import Diagnostics
from object_detection_tensorflow.srv import DetectObjects
from object_detection_tensorflow.msg import Detection


class ObjectDetectionNode(Node):

    def __init__(self,
                 saved_model_path,
                 label_map_path,
                 image_topic,
                 result_topic,
                 name="object_detection_tensorflow",
                 min_probability=0.5,
                 data_timeout=1.5,
                 max_gpu_memory_fraction=None,
                 result_image_size=(640, 480),
                 log_level=rclpy.logging.LoggingSeverity.INFO):

        super().__init__(name)

        self.logger = self.get_logger()

        self.collection_timeout = data_timeout
        self.min_probability = min_probability
        self.result_image_size = result_image_size

        self.detection = ObjectDetection(saved_model_path,
                                         label_map_path,
                                         max_gpu_memory_fraction)

        self.visualization = Visualization()

        self.subscriber = self.create_subscription(
            Image, image_topic, self._image_handler)

        self.publisher = self.create_publisher(
            Image, result_topic, 1)

        self.service = self.create_service(DetectObjects, "DetectObjects", self._detect_objects)

        self.bridge = CvBridge()

        self.image_msg = None

    def run(self):

        try:
            self.logger.info("Started service")

            rclpy.spin(self)

        except KeyboardInterrupt:
            self.logger.info("Stopped service")

        self.destroy_node()
        rclpy.shutdown()

    def _image_handler(self, msg: Image):

        self.image_msg = msg

    def _get_image(self):

        image = None
        image_header = None
        duration = 0

        self.image_msg = None

        start_time = self.get_clock().now()

        while image is None and duration < self.collection_timeout:
            if self.image_msg is not None and self.image_msg.header.stamp.to_time() >= start_time:
                try:
                    image = self.bridge.imgmsg_to_cv2(self.image_msg)
                    image_header = self.image_msg.header

                    # if grayscale image, stack it to comply to
                    # the 3 different channels of input tensor
                    if len(image.shape) == 2:
                        image = np.stack((image,)*3, axis=-1)

                except CvBridgeError as e:
                    self.logger.error(e)

            duration = self.get_clock().now() - start_time

        self.diagnostics.update("data_acquisition_time", duration)

        return image, image_header

    def _publish_image(self, image: np.ndarray):

        try:
            image = cv2.resize(image,
                               dsize=self.result_image_size,
                               interpolation=cv2.INTER_AREA)

            self.publisher.publish(self.bridge.cv2_to_imgmsg(
                image, encoding="bgr8"))

        except CvBridgeError as e:
            self.logger.error(e)

    def _detect_objects(self, request, response):

        self.logger.info("Detecting objects")

        detected_objects = None

        if request.roi is not None:
            if not any([request.roi.x_offset, request.roi.y_offset,
                        request.roi.width, request.roi.height]):
                request.roi = None

        try:
            image, image_header = self._get_image()

            start_time = self.get_clock().now()

            if image is not None:
                detected_objects = self._clear(
                    self._convert_to_ros(
                        self.detection.run(image, request.roi)))

                image = self.visualization.draw_detections(
                    image, detected_objects, request.roi)

                self.diagnostics.update(
                    "last_timestamp_processed", image_header.stamp.to_sec())
                self.diagnostics.update(
                    "detection_time", self.get_clock().now() - start_time)

                self._publish_image(image)

                self.logger.info("Detected {} objects".format(
                    len(detected_objects)))

                self.logger.debug(f"{detected_objects}")

            else:
                self.logger.error("Data could not be collected (timeout={}s)!".format(
                    self.collection_timeout))

        except tf.errors.InvalidArgumentError as e:
            self.logger.error("Caught {} during detection: {}".format(type(e), str(e)) +
                              "\nPlease make sure the given ROI dimensions fit the image size!")

        except Exception as e:
            self.logger.error("Caught {} during detection: {}".format(
                type(e), str(e)))

        # for detection in detected_objects:
        #     detection.header.stamp = self.get_clock().now().to_msg()
        #     detection.header.frame_id = "object_detection"

        response.detections = detected_objects
        response.image = image

        return response

    def _clear(self, detections):

        cleared_detections = []

        for detection in detections:
            if detection.probability >= self.min_probability:
                cleared_detections.append(detection)

        return cleared_detections

    def _convert_to_ros(self, raw_detections):


def main(args=None):

    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        description='Object Detection TensorFlow ROS node')
    parser.add_argument('-n', '--node_name', default="object_detection_tensorflow",
                        help="ROS node name")
    parser.add_argument('--image_topic', default="/stereo/left/image_rect_color",
                        help="ROS topic to listen for images")
    parser.add_argument('--saved_model_path', default="/saved_model",
                        help="Path to TensorFlow saved model folder")
    parser.add_argument('--label_map_path', default="/label_map.txt",
                        help="Text file with class names (one label per line)")
    parser.add_argument('--min_probability', type=float, default=0.99,
                        help="Minimum probability for detections to be reported")
    parser.add_argument('--data_acquisition_timeout', type=float, default=1.5,
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
    package_share_directory = get_package_share_directory("object_detection_tensorflow")
    saved_model_path = os.path.join(package_share_directory, "..", "..", "lib/object_detection_tensorflow/data/saved_model")
    label_map_path = os.path.join(package_share_directory, "..", "..", "lib/object_detection_tensorflow/data/label_map.txt")

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
