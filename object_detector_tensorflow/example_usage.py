#!/usr/bin/env python3
from object_detector_tensorflow.srv import DetectObjects

import rclpy
from rclpy.node import Node


class ExampleUsage(Node):

    def __init__(self):
        super().__init__('ExampleUsage')

        self.client = self.create_client(DetectObjects, 'DetectObjects')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DetectObjects.Request()

    def send_request(self):
        self.req.roi.y_offset = 100
        self.req.roi.x_offset = 100
        self.req.roi.height = 100
        self.req.roi.width = 100
        self.future = self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    node = ExampleUsage()
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
