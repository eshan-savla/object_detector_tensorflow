#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Image

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow.msg import Detections


class ContinuousDetectionNode(ObjectDetectionBaseNode):

    def __init__(self, node_name="continuous_detection_node"):

        super().__init__(node_name)

        self.subscriber = self.create_subscription(
            Image, self.image_topic, self._detect_objects)

        self.image_publisher = self.create_publisher(
            Image, f"{node_name}/result_image", 1)

        self.detections_publisher = self.create_publisher(
            Detections, f"{node_name}/detections", 1)

    def _detect_objects(self, msg: Image):

        detected_objects, result_image = super()._detect_objects(msg)

        self.image_publisher.publish(result_image)

        self.detections_publisher.publish(detected_objects)


def main(args=None):

    rclpy.init(args=args)

    ContinuousDetectionNode().run()


if __name__ == '__main__':
    main()
