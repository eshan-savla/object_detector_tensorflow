#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Image

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow.msg import Detections
from object_detector_tensorflow.srv import DetectObjects


class DetectionNode(ObjectDetectionBaseNode):

    def __init__(self, node_name="detection_node"):

        super().__init__(node_name)

        self.service = self.create_service(
            DetectObjects, f"{node_name}/detect_objects", self._detect_objects)

        self.image_publisher = self.create_publisher(
            Image, f"{node_name}/result_image", 1)

        self.detections_publisher = self.create_publisher(
            Detections, f"{node_name}/detections", 1)

    def _detect_objects(self, request, response):

        detected_objects, result_image = super()._detect_objects(
            request.image, request.roi)

        response.detections = detected_objects
        response.result_image = result_image

        self.image_publisher.publish(result_image)

        self.detections_publisher.publish(detected_objects)

        return response


def main(args=None):

    rclpy.init(args=args)

    DetectionNode().run()


if __name__ == '__main__':
    main()
