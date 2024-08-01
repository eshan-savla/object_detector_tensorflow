#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class ImagePublisher(Node):
	def __init__(self):
		super().__init__('image_publisher')
		self.rgb_publisher = self.create_publisher(Image, 'rgb_image', 10)
		self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)
		self.timer = self.create_timer(1, self.timer_callback)
		self.bridge = CvBridge()

	def timer_callback(self):
		# Create a dummy RGB image
        # Check if file exists at path
		rgb_image = cv2.imread("/home/docker/ros2_ws/src/object_detector_tensorflow/ros/object_detector_tensorflow/data/test_rgb.png")
		self._logger.info(f"RGB image shape: {rgb_image.shape}")
		# rgb_image[:, :, 0] = 255  # Red channel

		# Create a dummy depth image
		depth_image = cv2.imread("/home/docker/ros2_ws/src/object_detector_tensorflow/ros/object_detector_tensorflow/data/test_depth.png", cv2.CV_8UC1)
		self._logger.info(f"Depth image shape: {depth_image.shape}")
		# depth_image[240, 320] = 1.0  # Set a single point to 1.0

		# Convert images to ROS Image messages
		rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="passthrough")
		depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")

		# Publish the images
		self.rgb_publisher.publish(rgb_msg)
		self.depth_publisher.publish(depth_msg)

		self.get_logger().info('Publishing RGB and Depth images')

def main(args=None):
	rclpy.init(args=args)
	node = ImagePublisher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()