#!/usr/bin/env python3

from time import sleep
import threading
import rclpy
import rclpy.executors
import rclpy.time
from sensor_msgs.msg import Image

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow_interfaces.msg import Detections
from object_detector_tensorflow_interfaces.srv import DetectObjects

class DetectionNode(ObjectDetectionBaseNode):

    def __init__(self,
                 node_name: str = "detection_node") -> None:

        super().__init__(node_name)

        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.service = self.create_service(
            DetectObjects, f"{node_name}/detect_objects", self._detect_objects, callback_group=self.service_callback_group)
        
        self.image_lock = threading.Lock()
        self.latest_img: Image = None
        self.depth_lock = threading.Lock()
        self.latest_depth: Image = None
        

        self.subscriber_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.image_subscriber = self.create_subscription(Image, self.image_topic, self._image_callback, 1, callback_group=self.subscriber_callback_group)
        self.depth_subscriber = self.create_subscription(Image, self.depth_image_topic, self._depth_callback, 1, callback_group=self.subscriber_callback_group)

        self.image_publisher = self.create_publisher(
            Image, f"{node_name}/result_image", 1)

        self.detections_publisher = self.create_publisher(
            Detections, f"{node_name}/detections", 1)

    def _image_callback(self, msg: Image) -> None:
        if self.latest_img is None:
            self.get_logger().info("Received image.")
        with self.image_lock:
            self.latest_img = msg

    def _depth_callback(self, msg: Image) -> None:
        if self.latest_depth is None:
            self.get_logger().info("Received depth image.")
        with self.depth_lock:
            self.latest_depth = msg

    def _detect_objects(self,
                        request: DetectObjects.Request,
                    response: DetectObjects.Response) -> DetectObjects.Response:
        self.get_logger().info("Received request to detect objects.")
        while self.latest_img is None or self.latest_depth is None:
            if self.latest_img is None:
                self.get_logger().info("Waiting for image...", throttle_duration_sec=2.0)    
            if self.latest_depth is None:
                self.get_logger().info("Waiting for depth image...", throttle_duration_sec=2.0)
        response.reference_image = self.latest_depth
        detected_objects, result_image = super()._detect_objects(
            self.latest_img, request.roi)
        response.detections = detected_objects
        response.result_image = result_image

        self.image_publisher.publish(result_image)

        self.detections_publisher.publish(detected_objects)
        self.get_logger().info("Published detections and result image.")
        with self.image_lock:
            self.latest_img = None
        with self.depth_lock:
            self.latest_depth = None
        self.get_logger().info("Reset latest images.")
        return response


def main(args=None) -> None:

    rclpy.init(args=args)

    # DetectionNode().run()

    # rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=3)

    node = DetectionNode()
    executor.add_node(node)

    try:
        node.logger.info("Started Node")

        executor.spin()

    except KeyboardInterrupt:
        node.logger.info("Stopped Node")

    executor.shutdown()
    node.destroy_node()



if __name__ == '__main__':
    main()
