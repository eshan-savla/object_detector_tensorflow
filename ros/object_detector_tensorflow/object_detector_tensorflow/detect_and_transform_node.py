#!/usr/bin/env python3

from time import sleep
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, TransformStamped
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from object_detector_tensorflow.base_node import ObjectDetectionBaseNode
from object_detector_tensorflow_interfaces.msg import Detections, Detection
from object_detector_tensorflow_interfaces.srv import DetectObjectPosition, DetectObjectPositions
from point_transformation_interfaces.srv import PixelToPoint


class DetectAndTransformNode(ObjectDetectionBaseNode):

    def __init__(self,
                 node_name: str = "detect_and_transform_node") -> None:

        super().__init__(node_name)

        self.image: Image = None
        self.depth_image: Image = None
        self.detected_objects: Detections = None
        self.lock_image = threading.Lock()
        self.lock_depth_image = threading.Lock()
        self.lock_detected_objects = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.external_group = MutuallyExclusiveCallbackGroup()
        self.depth_image_group = MutuallyExclusiveCallbackGroup()
        self.transform_service_group = MutuallyExclusiveCallbackGroup()

        self.service = self.create_service(DetectObjectPosition,
                                           f"{node_name}/detect_object_and_transform",
                                           self.detect_object_and_transform)
        
        self.service_2 = self.create_service(DetectObjectPositions,
                                             f"{node_name}/detect_objects_and_transform",
                                            self.detect_objects_and_transform)

        self.image_publisher = self.create_publisher(Image, f"{node_name}/result_image", 1,
                                                     callback_group=self.external_group)
        self.detections_publisher = self.create_publisher(Detections, f"{node_name}/detections", 1,
                                                          callback_group=self.external_group)

        self.image_subscription = self.create_subscription(Image, self.image_topic,
                                                           self._image_callback, 1,
                                                           callback_group=self.external_group)
        self.depth_image_subscription = self.create_subscription(Image, self.depth_image_topic,
                                                                 self._depth_image_callback, 1,
                                                                 callback_group=self.depth_image_group)

        self.transform_client = self.create_client(PixelToPoint,
                                                   'point_transformation_node/pixel_to_point',
                                                   callback_group=self.transform_service_group)

        self.tf_publisher = tf2_ros.TransformBroadcaster(self)

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
                self.get_logger().warn("Stuff was recieved and is not none")
                with self.lock_image:
                    current_image = self.image

                with self.lock_depth_image:
                    current_depth_image = self.depth_image

                with self.lock_detected_objects:
                    current_detected_objects = self.detected_objects

                self.get_logger().info("detection recieved")

                return (current_image, current_depth_image, current_detected_objects)

            else:
                if self.image is None:
                    self.get_logger().warn('no image', throttle_duration_sec=2.0)
                if self.depth_image is None:
                    self.get_logger().warn('no depth image', throttle_duration_sec=2.0)
                if self.detected_objects is None:
                    self.get_logger().warn('no detection', throttle_duration_sec=2.0)
                self.get_logger().info('sleep', throttle_duration_sec=2.0)
                sleep(0.1)

        raise Exception("No new Detection received")

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

    def _transform_point_to_base_frame(self, base_frame: str, point: Point) -> Point:
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera'
        t.child_frame_id = 'detection'
        t.transform.translation.x = point.x
        t.transform.translation.y = point.y
        t.transform.translation.z = point.z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_publisher.sendTransform(t)

        while rclpy.ok():
            if self.tf_buffer.can_transform(
                base_frame,
                "detection",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            ):
                self.get_logger().info("transform possible")
                try:
                    t_detection = self.tf_buffer.lookup_transform(
                        base_frame,
                        "detection",
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=1.0)
                    )
                except tf2_ros.TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {base_frame} to detection: {ex}')
                    return Point()
                self.get_logger().info("transform done")
                break
            else:
                self.get_logger().info("transform waiting...")
                sleep(0.1)

        # transform = tf2_ros.TransformStamped()
        # transform = tf2_ros.convert(pose, tf2_ros.TransformStamped)

        # transformed_pose = self.tf_buffer.transform(pose,
        #                                             base_frame,
        #                                             timeout=rclpy.duration.Duration(seconds=1.0))

        # transformed_point = tf2_ros.convert(transformed_pose, PointStamped)

        transformed_point = Point()
        transformed_point.x = t_detection.transform.translation.x
        transformed_point.y = t_detection.transform.translation.y
        transformed_point.z = t_detection.transform.translation.z

        return transformed_point
    
    def _get_detected_position(self, object: Detection, image: Image, depth_image: Image) -> Detection:
        detected_position = Detection()
        detected_position.class_name = object.class_name
        detected_position.probability = object.probability

        # Transform pixel to point
        tranform_request = PixelToPoint.Request()
        tranform_request.pixels = [object.center]
        tranform_request.height = image.height
        tranform_request.width = image.width
        tranform_request.depth_image = depth_image
        tranform_request.camera_type = "camera"

        transform_response: PixelToPoint.Response = self.transform_client.call(tranform_request)

        # Transform point from camera frame to base_frame
        point_transformed = self._transform_point_to_base_frame("base_link", transform_response.points[0])

        detected_position.center = point_transformed
        self.get_logger().info(f"Returning detected transformed points: {detected_position.center}")
        return detected_position

    def detect_object_and_transform(self,
                                    request: DetectObjectPosition.Request,
                                    response: DetectObjectPosition.Response) -> DetectObjectPosition.Response:

        image, depth_image, detected_objects = self._wait_for_new_detection()
        self.logger.warn(f"Got images and detections")
        # image, depth_image, detected_objects = self._get_example_data()

        # Filter detected objects by class name
        if not request.class_name == "":
            detected_objects.detections = [
                detection for detection in detected_objects.detections if detection.class_name == request.class_name]

        if len(detected_objects.detections) == 0:
            self.logger.info(f"No object of class '{request.class_name}' found")
            response.class_name = f"No object of class '{request.class_name}' found"
            response.probability = 0.0
            return response

        max_prob_object = max(detected_objects.detections, key=lambda x: x.probability)

        detected_position = self._get_detected_position(max_prob_object, image, depth_image)
        # # Transform pixel to point
        # tranform_request = PixelToPoint.Request()
        # tranform_request.pixels = [max_prob_object.center]
        # tranform_request.height = image.height
        # tranform_request.width = image.width
        # tranform_request.depth_image = depth_image
        # tranform_request.camera_type = request.camera_type

        # transform_response: PixelToPoint.Response = self.transform_client.call(tranform_request)

        # # Transform point from camera frame to base_frame
        # point_transformed = self._transform_point_to_base_frame(request.base_frame, transform_response.points[0])

        # Return transformed point
        response.point = detected_position.center
        response.class_name = max_prob_object.class_name
        response.probability = max_prob_object.probability
        self.logger.info(
            f"Found object '{response.class_name}' with probability {response.probability} at {response.point}")
        self._reset_images()
        return response
    
    def detect_objects_and_transform(self, request: DetectObjectPositions.Request, response: DetectObjectPositions.Response) -> DetectObjectPositions.Response:
        image, depth_image, detected_objects = self._wait_for_new_detection()

        if not len(request.class_names) == 0:
            detected_objects.detections = [
                detection for detection in detected_objects.detections if detection.class_name in request.class_names] 
        
        if len(detected_objects.detections) == 0:
            self.logger.info(f"No object of classes '{request.class_names}' found")
            response.class_names = f"No object of class '{request.class_names}' found"
            return response

        for detected_object in detected_objects.detections:
            detected_object.center = (self._get_detected_position(detected_object, image, depth_image)).center
            self.get_logger().info(f"Got new position of detected object in detections")
        
        response.class_names = request.class_names
        response.detected_positions = detected_objects.detections
        self._reset_images()
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
