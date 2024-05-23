#!/usr/bin/env python3

from time import sleep
import rclpy
import rclpy.time
from sensor_msgs.msg import Image

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow_interfaces.msg import Detections
from object_detector_tensorflow_interfaces.srv import DetectObjects


class DetectionNode(ObjectDetectionBaseNode):

    def __init__(self,
                 node_name: str = "detection_node") -> None:

        super().__init__(node_name)

        self.service = self.create_service(
            DetectObjects, f"{node_name}/detect_objects", self._detect_objects)
        
        self.latest_img: Image = None

        self.image_subscriber = self.create_subscription(Image, self.image_topic, self._subscriber_callback, 1)

        self.image_publisher = self.create_publisher(
            Image, f"{node_name}/result_image", 1)

        self.detections_publisher = self.create_publisher(
            Detections, f"{node_name}/detections", 1)

    def _subscriber_callback(self, msg: Image) -> None:
        self.latest_img = msg

    def _detect_objects(self,
                        request: DetectObjects.Request,
                    response: DetectObjects.Response) -> DetectObjects.Response:
        while self.latest_img is None:
            self.get_logger().info("Waiting for image...", throttle_duration_sec=2.0)
            sleep(0.1)
        response.reference_image = self.latest_img
        detected_objects, result_image = super()._detect_objects(
            self.latest_img)
        response.detections = detected_objects
        response.result_image = result_image

        self.image_publisher.publish(result_image)

        self.detections_publisher.publish(detected_objects)
        self.latest_img = None
        return response


def main(args=None) -> None:

    rclpy.init(args=args)

    DetectionNode().run()


if __name__ == '__main__':
    main()
