#!/usr/bin/env python3

import string
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from object_detector_tensorflow_interfaces.srv import DetectObjectPositions
from object_detector_tensorflow_interfaces.msg import Detection


class Client():

    def __init__(self,
                 node: Node) -> None:

        self.node = node

        self.client = self.node.create_client(
            DetectObjectPositions, 'detect_and_transform_node/detect_objects_and_transform')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('DetectObjectPositions service not available, waiting again...')

        self.request = DetectObjectPositions.Request()

    def detect_objects(self,
                       base_frame: str,
                       camera_type: str) -> Tuple[str, List[Detection]]:

        self.request.base_frame = base_frame
        self.request.camera_type = camera_type    
        future = self.client.call_async(self.request)
        try:
            rclpy.spin_until_future_complete(self.node, future)

        except KeyboardInterrupt:
            self.node.get_logger().info("KeyboardInterrupt")

        response:DetectObjectPositions.Response = future.result()

        self.node.get_logger().info(f"Got transformed detections of the following items {response.class_names}")

        return response.class_names, response.detected_positions


def main(args=None) -> None:

    rclpy.init(args=args)

    node = rclpy.create_node('detect_transform_test_node')

    client = Client(node)

    classes, detections = client.detect_objects("world", "roboception")

    print(classes)
    print(detections)

    #####################

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
