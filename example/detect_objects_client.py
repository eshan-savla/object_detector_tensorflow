#!/usr/bin/env python3
from object_detector_tensorflow.srv import DetectObjects

import rclpy
from rclpy.node import Node


class DetectObjectsClient(Node):

    def __init__(self):
        super().__init__('detect_objects_client')

        self.client = self.create_client(DetectObjects, 'single_detection_node/detect_objects')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = DetectObjects.Request()

    def send_request(self):
        self.request.roi.y_offset = 100
        self.request.roi.x_offset = 100
        self.request.roi.height = 100
        self.request.roi.width = 100
        self.future = self.client.call_async(self.request)


def main(args=None):
    rclpy.init(args=args)

    node = DetectObjectsClient()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            response = node.future.result()
            node.get_logger().info(
                str(response.detections))
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
