#!/usr/bin/env python3
from iris_ros_core.srv import DetectObjects
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class DetectObjectsClient(Node):

    def __init__(self):
        super().__init__('detect_objects_client')

        self.bridge = CvBridge()

        self.client = self.create_client(DetectObjects, 'single_detection_node/detect_objects')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = DetectObjects.Request()

    def send_request(self):

        image = np.zeros([960, 1280, 3], dtype=np.uint8)

        self.request.image = self.bridge.cv2_to_imgmsg(
            image, encoding="bgr8")

        self.request.roi.y_offset = 100
        self.request.roi.x_offset = 100
        self.request.roi.height = 100
        self.request.roi.width = 100

        self.future = self.client.call_async(self.request)

    def response_recieved(self, response):
        self.get_logger().info(str(response.detections.detections))

        result_image = self.bridge.imgmsg_to_cv2(response.result_image)
        cv2.imshow("result_image", result_image)
        cv2.waitKey(0)


def main(args=None):
    rclpy.init(args=args)

    node = DetectObjectsClient()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            response = node.future.result()
            node.response_recieved(response)
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
