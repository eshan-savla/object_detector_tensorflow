#!/usr/bin/env python3

import rclpy
from ros_core.srv import DetectObjects

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode


class DetectionNode(ObjectDetectionBaseNode):

    def __init__(self, node_name="detection_node"):

        super().__init__(node_name)

        self.service = self.create_service(
            DetectObjects, f"{node_name}/detect_objects", self._detect_objects)

    def _detect_objects(self, request, response):

        detected_objects, result_image = super()._detect_objects(
            request.image, request.roi)

        response.detections = detected_objects
        response.result_image = result_image

        return response


def main(args=None):

    rclpy.init(args=args)

    DetectionNode().run()


if __name__ == '__main__':
    main()
