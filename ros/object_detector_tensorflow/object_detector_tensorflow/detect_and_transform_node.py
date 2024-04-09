#!/usr/bin/env python3

from time import sleep
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Transform
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow_interfaces.msg import Detections, Detection
from object_detector_tensorflow_interfaces.srv import DetectObjectPosition
from point_transformation_interfaces.srv import PixelToPoint


class DetectAndTransformNode(ObjectDetectionBaseNode):

    def __init__(self,
                 node_name: str = "detect_and_transform_node") -> None:

        super().__init__(node_name)

        self.image: Image = None
        self.depth_image: Image = None
        self.detected_objects = None
        self.lock_image = threading.Lock()
        self.lock_depth_image = threading.Lock()
        self.lock_detected_objects = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.external_group = MutuallyExclusiveCallbackGroup()
        self.depth_image_group = MutuallyExclusiveCallbackGroup()

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
                                                                 callback_group=self.depth_image_group)

        self.transform_client = self.create_client(PixelToPoint,
                                                   'point_transformation_node/pixel_to_point',
                                                   callback_group=self.external_group)

    def _image_callback(self, msg: Image) -> None:
        detected_objects, result_image = super()._detect_objects(msg)

        with self.lock_image:
            self.image = msg

        with self.lock_detected_objects:
            self.detected_objects = detected_objects

        self.image_publisher.publish(result_image)
        self.detections_publisher.publish(detected_objects)

    def _depth_image_callback(self, msg):
        with self.lock_depth_image:
            self.depth_image = msg

    def _reset_images(self):
        with self.lock_image:
            self.image = None

        with self.lock_depth_image:
            self.depth_image = None

        with self.lock_detected_objects:
            self.detected_objects = None

    def _wait_for_new_detection(self) -> tuple[Image, Image, Detections]:
        # self._reset_images()

        while rclpy.ok():
            if self.image is not None and self.depth_image is not None and self.detected_objects is not None:
                with self.lock_image:
                    current_image = self.image

                with self.lock_depth_image:
                    current_depth_image = self.depth_image

                with self.lock_detected_objects:
                    current_detected_objects = self.detected_objects

                self.get_logger().info("detection recieved")

                return (current_image, current_depth_image, current_detected_objects)

            else:
                self.get_logger().info('sleep')
                sleep(0.1)

        return (None, None, None)

    def _get_example_data(self):
        ####################
        # Generate test data
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
        image = bridge.cv2_to_imgmsg(np.zeros([960, 1280, 3], dtype=np.uint8), encoding="bgr8")
        depth_image = bridge.cv2_to_imgmsg(np.zeros([480, 640, 3], dtype=np.float32), encoding="32FC3")

        return image, depth_image, detected_objects
        ####################

    def _transform_point_to_base_frame(self, base_frame: str, pose: Transform) -> Point:
        # Transformation from camera to world is necessary
        # Add this to the launch file and change parameters
        #
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0', '0', '1', '0', '0', '0', 'world', 'camera']
        # ),

        try:
            t = self.tf_buffer.lookup_transform(
                base_frame,
                "camera",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {base_frame} to camera: {ex}')
            return Point()

        print(t)

        # pose = self.tf_buffer.transform(pose,
        #                                 base_frame,
        #                                 timeout=rclpy.duration.Duration(seconds=1.0))

        # print(pose)

        transformed_point = Point()
        transformed_point.x = t.transform.translation.x
        transformed_point.y = t.transform.translation.y
        transformed_point.z = t.transform.translation.z

        return transformed_point

    def detect_object_and_transform(self,
                                    request: DetectObjectPosition.Request,
                                    response: DetectObjectPosition.Response) -> None:

        image, depth_image, detected_objects = self._wait_for_new_detection()
        # image, depth_image, detected_objects = self._get_example_data()

        # Filter detected objects by class name
        if not request.class_name == "":
            detected_objects.detections = [
                detection for detection in detected_objects.detections if detection.class_name == request.class_name]

        if len(detected_objects.detections) == 0:
            self.logger.info(f"No object of class '{request.class_name}' found")
            response.class_name = "No object of class '{request.class_name}' found"
            response.probability = 0.0
            return response

        max_prob_object = max(detected_objects.detections, key=lambda x: x.probability)

        # Transform pixel to point
        tranform_request = PixelToPoint.Request()
        tranform_request.pixels = [max_prob_object.center]
        tranform_request.height = image.height
        tranform_request.width = image.width
        tranform_request.depth_image = depth_image
        tranform_request.camera_type = request.camera_type

        transform_response: PixelToPoint.Response = self.transform_client.call(tranform_request)

        # Transform point from camera frame to base_frame
        pose_stamped = Transform()
        # pose_stamped.header = image.header
        pose_stamped.translation.x = transform_response.points[0].x
        pose_stamped.translation.y = transform_response.points[0].y
        pose_stamped.translation.z = transform_response.points[0].z
        point_transformed = self._transform_point_to_base_frame(request.base_frame, pose_stamped)

        # Return transformed point
        response.point = point_transformed
        response.class_name = max_prob_object.class_name
        response.probability = max_prob_object.probability
        self.logger.info(
            f"Found object '{response.class_name}' with probability {response.probability} at {response.point}")

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
