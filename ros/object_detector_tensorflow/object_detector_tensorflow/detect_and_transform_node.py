#!/usr/bin/env python3

from math import e
from platform import node
from time import sleep

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow_interfaces.msg import Detections, Detection
from object_detector_tensorflow_interfaces.srv import DetectObjectPosition
from point_transformation_interfaces.srv import PixelToPoint


class DetectAndTransformNode(ObjectDetectionBaseNode):

    def __init__(self,
                 node_name: str = "detect_and_transform_node") -> None:

        super().__init__(node_name)

        self.image : Image = None 
        self.depth_image : Image = None
        self._current_image : Image = None
        self._current_depth_image : Image = None

        self.external_group = MutuallyExclusiveCallbackGroup()

        self.service = self.create_service(DetectObjectPosition,
                                           f"{node_name}/detect_object_and_transform", 
                                           self.detect_object_and_transform)

        self.image_publisher = self.create_publisher(Image, f"{node_name}/result_image", 1,
                                           callback_group=self.external_group)
        self.detections_publisher = self.create_publisher(Detections, f"{node_name}/detections", 1,
                                           callback_group=self.external_group)
        
        self.image_subscription = self.create_subscription(Image, self.image_topic, 
                                                           self._image_callback, 10, 
                                                           callback_group=self.external_group)
        self.depth_image_subscription = self.create_subscription(Image, self.depth_image_topic, 
                                                                 self._depth_image_callback, 10, 
                                                                 callback_group=self.external_group)
        
        self.transform_client = self.create_client(PixelToPoint, 
                                                   'point_transformation_node/pixel_to_point',
                                                   callback_group=self.external_group)
        
    def _image_callback(self, msg):

        self._current_image = msg

    def _depth_image_callback(self, msg):

        self._current_depth_image = msg

    def _images_recieved(self):
        if self._current_image is not None and self._current_depth_image is not None:
            self.image = self._current_image
            self.depth_image = self._current_depth_image

            print("images recieved")

            return True

        return False

    def _wait_for_new_images(self):
        self._current_image = None
        self._current_depth_image = None

        while rclpy.ok() and not self._images_recieved():
            sleep(0.1)

    def detect_object_and_transform(self,
                        request: DetectObjectPosition.Request,
                        response: DetectObjectPosition.Response) -> None:
        # self._wait_for_new_images()

        # detected_objects, result_image = super()._detect_objects(self.image, None)

        ############# Generate test data
        example_objects = [
            {"class_id": 1, "class_name": "cat", "probability": 0.9, "bounding_box": (10, 20, 30, 40)},
            {"class_id": 2, "class_name": "dog", "probability": 0.8, "bounding_box": (50, 60, 70, 80)},
            {"class_id": 3, "class_name": "bird", "probability": 0.7, "bounding_box": (90, 100, 110, 120)}
        ]

        detected_objects = Detections()
        for obj in example_objects:
            detection = Detection()
            detection.class_id = obj["class_id"]
            detection.class_name = obj["class_name"]
            detection.probability = obj["probability"]
            detection.bounding_box.x_offset = obj["bounding_box"][0]
            detection.bounding_box.y_offset = obj["bounding_box"][1]
            detection.bounding_box.width = obj["bounding_box"][2]
            detection.bounding_box.height = obj["bounding_box"][3]
            detected_objects.detections.append(detection)

        from cv_bridge import CvBridge
        import numpy as np
        bridge = CvBridge()
        # img = cv2.imread("/home/docker/ros2_ws/src/object_detector_tensorflow/ros/object_detector_tensorflow/data/test_image.jpeg", 0) 
        self.image  = bridge.cv2_to_imgmsg(np.zeros([960, 1280, 3], dtype=np.uint8), encoding="bgr8")
        self.depth_image  = bridge.cv2_to_imgmsg(np.zeros([480, 640, 3], dtype=np.float32), encoding="32FC3")
        result_image = self.image
        ##############

        self.image_publisher.publish(result_image)
        self.detections_publisher.publish(detected_objects)

        if not request.class_name == "":  
            detected_objects.detections = [detection for detection in detected_objects.detections if detection.class_name == request.class_name]

        if len(detected_objects.detections) == 0:
            self.logger.info(f"No object of class '{request.class_name}' found")
            response.class_name = "No object of class '{request.class_name}' found"
            response.probability = 0.0
            return response

        max_prob_object = max(detected_objects.detections, key=lambda x: x.probability)

        center_pixel = Point()
        center_pixel.x = float(max_prob_object.bounding_box.x_offset + int(max_prob_object.bounding_box.width / 2))
        center_pixel.y = float(max_prob_object.bounding_box.y_offset + int(max_prob_object.bounding_box.height / 2))
        
        tranform_request = PixelToPoint.Request()
        tranform_request.pixels = [center_pixel]
        tranform_request.height = self.image.height
        tranform_request.width = self.image.width
        tranform_request.depth_image = self.depth_image
        tranform_request.camera_type = request.camera_type

        transform_response = self.transform_client.call(tranform_request)

        # Transform point with tf from camera frame to base_frame

        response.point = transform_response.points[0]
        response.class_name = max_prob_object.class_name
        response.probability = max_prob_object.probability
        self.logger.info(f"Found object '{response.class_name}' with probability {response.probability} at {response.point}")
        
        return response


def main(args=None) -> None:

    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=3)

    node = DetectAndTransformNode()
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
