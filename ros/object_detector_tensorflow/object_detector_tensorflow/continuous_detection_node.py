#!/usr/bin/env python3

import threading
import rclpy
import rclpy.executors
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow_interfaces.msg import Detections


class ContinuousDetectionNode(ObjectDetectionBaseNode):

    def __init__(self,
                 node_name: str = "continuous_detection_node",
                 keep_last: int = 1) -> None:
        super().__init__(node_name)
        self.declare_parameter("detect_hz", 0.1)
        self.detect_hz = self.get_parameter("detect_hz").get_parameter_value().double_value
        self.subscriber_group = MutuallyExclusiveCallbackGroup()
        self.publisher_group = MutuallyExclusiveCallbackGroup()
        self.image_lock = threading.Lock()
        self._latest_img:Image = None
        self.subscriber = self.create_subscription(
            Image, self.image_topic, self._subscriber_callback, keep_last, callback_group=self.subscriber_group)

        self.create_timer(1/self.detect_hz, self._timed_detect_objects, callback_group=self.subscriber_group)
        self.image_publisher = self.create_publisher(
            Image, f"{node_name}/result_image", 1, callback_group=self.publisher_group)

        self.detections_publisher = self.create_publisher(
            Detections, f"{node_name}/detections", 1, callback_group=self.publisher_group)

    def _subscriber_callback(self, msg: Image) -> None:
        with self.image_lock:
            self._latest_img = msg

    def _timed_detect_objects(self) -> None:
        with self.image_lock:
            if self._latest_img is not None:
                detected_objects, result_image = super()._detect_objects(self._latest_img)
                self.image_publisher.publish(result_image)

                self.detections_publisher.publish(detected_objects)
                self._latest_img = None


def main(args=None):

    rclpy.init(args=args)

    ContinuousDetectionNode().run()


if __name__ == '__main__':
    main()
