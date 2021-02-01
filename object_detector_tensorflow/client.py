#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, RegionOfInterest
from iris_ros_core.srv import DetectObjects


class Client():

    def __init__(self, node: Node):

        self.node = node

        self.client = self.node.create_client(DetectObjects, 'detection_node/detect_objects')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        self.request = DetectObjects.Request()

    def detect_objects_async(self, image: Image, roi: RegionOfInterest):

        self.request.image = image
        self.request.roi = roi

        return self.client.call_async(self.request)

    def detect_objects(self, image: Image, roi: RegionOfInterest):

        future = self.detect_objects_async(image, roi)

        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()

        self.node.get_logger().info(str(response.detections.detections))

        return response.detections.detections, self.bridge.imgmsg_to_cv2(response.result_image)


def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node('example_node')

    ### Usage example ###

    client = Client(node)

    # Image collected via camera driver, e.g. rc_visard_ros
    image = Image(witdth=1280,
                  height=960)

    # (Optional) Only search objects in this region of interest
    roi = RegionOfInterest(x_offset=100,
                           y_offset=100,
                           height=100,
                           width=100)

    detections, result_image = client.detect_objects(image, roi)

    #####################

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
