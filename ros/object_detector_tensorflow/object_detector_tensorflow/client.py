#!/usr/bin/env python3

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Image, RegionOfInterest

from object_detector_tensorflow_interfaces.srv import DetectObjects
from object_detector_tensorflow_interfaces.msg import Detection


class Client():

    def __init__(self,
                 node: Node) -> None:

        self.node = node

        self.client = self.node.create_client(
            DetectObjects, 'detection_node/detect_objects')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('DetectObjects service not available, waiting again...')

        self.request = DetectObjects.Request()

    def detect_objects_async(self,
                             image: Image,
                             roi: RegionOfInterest) -> Future:

        self.request.image = image
        self.request.roi = roi

        return self.client.call_async(self.request)

    def detect_objects(self,
                       image: Image,
                       roi: RegionOfInterest) -> Tuple[List[Detection], Image]:

        future = self.detect_objects_async(image, roi)

        try:
            rclpy.spin_until_future_complete(self.node, future)

        except KeyboardInterrupt:
            self.node.get_logger().info("KeyboardInterrupt")

        response = future.result()

        self.node.get_logger().info(str(response.detections.detections))

        return response.detections.detections, response.result_image


def main(args=None) -> None:

    rclpy.init(args=args)

    node = rclpy.create_node('example_node')

    ### Usage example ###
    from cv_bridge import CvBridge
    import numpy as np
    import cv2

    bridge = CvBridge()
    client = Client(node)

    # Test image empty
    # img = np.zeros([960, 1280, 3], dtype=np.uint8)

    # Test image from folder
    img = cv2.imread('/home/docker/ros2_ws/src/object_detector_tensorflow/ros/object_detector_tensorflow/data/test_image.png', 0) 
    image = bridge.cv2_to_imgmsg(img)

    # Image collected via camera driver, e.g. rc_visard_ros
    # imgage = Image(height=640, width=480, encoding="rgb8")

    # (Optional) Only search objects in this region of interest
    roi = RegionOfInterest(x_offset=0,
                           y_offset=0,
                           height=960,
                           width=1280)

    detections, result_image = client.detect_objects(image, roi)

    print(detections)
    print(result_image.width, result_image.height)

    image = bridge.imgmsg_to_cv2(result_image)
    cv2.imshow("result", image)
    cv2.waitKey(5000)

    #####################

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
