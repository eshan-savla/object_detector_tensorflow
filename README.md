# Object Detector Tensorflow

ROS2 Nodes for TensorFlow Object Detection Inference

## How to start:
1. Place the tensorflow saved_model folder into the `./ros/object_detector_tensorflow/data/` path
2. Write the correct names of possible classes in the file `./ros/object_detector_tensorflow/data/label_map.txt`
   ```yaml
   class1 #label id = 0
   class2 #label id = 1
   ```
3. Set the correct ROS_DOMAIN_ID in `./.env`
    ```yaml
    ROS_DOMAIN_ID=66
    ```
4. (only once) Build the docker container
   ```bash
   . build_docker.sh
   ```
5. Start the docker container
   ```bash
   . start_docker.sh
   ```
6. Launch the ros node
   ```bash
   ros2 launch object_detector_tensorflow detection.launch.py
   ```

## How to test:
1. Place a test image in this folder: `./ros/object_detector_tensorflow/data/test_image.png`
2. Or change input type in this file: `./ros/object_detector_tensorflow/object_detector_tensorflow/client.py`
    ```python
    # Test image empty
    img = np.zeros([960, 1280, 3], dtype=np.uint8)

    # Test image from folder
    img = cv2.imread('/home/docker/ros2_ws/src/object_detector_tensorflow/ros/object_detector_tensorflow/data/test_image.png', 0) 
    image = bridge.cv2_to_imgmsg(img)

    # Image collected via camera driver, e.g. rc_visard_ros
    imgage = Image(height=640, width=480, encoding="rgb8")
    ```
3. Launch detection_node
    ```bash
    ros2 launch object_detector_tensorflow detection.launch.py
    ```
4. Run client node
    ```bash
    ros2 run object_detector_tensorflow client
    ```

Test `detect_and_transform_node`
```bash
ros2 launch object_detector_tensorflow detect_and_transform.launch.py
ros2 service call /detect_and_transform_node/detect_object_d_transform object_detector_tensorflow_interfaces/srv/DetectObjectPosition "{class_name: '', base_frame: 'base', camera_type: 'roboception'}"
```

## Interface:

### Continuous detection node

- `continuous_detection_node` (ROS2 Dashing python)

    Node used for continuous stream of image data

![continuous_detection_node](docs/continuous_detection_node.svg)

#### Topics

- subscribe `image` (sensor_msgs/Image)
- publish `continuous_detection_node/result_image` (sensor_msgs/Image)
- publish `continuous_detection_node/detections` (custom type)

    ```bash
    # Detections.msg
    std_msgs/Header header
    std_msgs/Header image_header
    Detection[] detections
    ```
    
    ```bash
    # Detection.msg
    uint32 class_id
    string class_name
    float32 probability
    sensor_msgs/RegionOfInterest bounding_box
    ```

#### Parameters

Change in `ros/object_detector_tensorflow/config/params.yaml`

```bash
saved_model_path: "data/saved_model"  # Path to TensorFlow saved model folder
label_map_path: "data/label_map.txt"  # Text file with class names (one label per line)
image_topic: "/image"  # ROS topic to listen for images
min_probability: 0.5            # Minimum probability for detections to be reported
max_gpu_memory_fraction: 1.0    # Limits the GPU memory usage of the TensorFlow model to only a fraction (between 0 and 1)
result_image_size: [640,480]    # Dimensions of the result image [x,y]
```

---

### Detection Node

- `detection_node` (ROS2 Dashing python)

    Node used for requests of object detection on single images

![detection_node](docs/detection_node.svg)

#### Topics

- publish `continuous_detection_node/result_image` (sensor_msgs/Image)
- publish `continuous_detection_node/detections` (custom type)

#### Services

- server `continuous_detection_node/detect_objects` (custom type)

    ```bash
    # DetectObjects.srv
    sensor_msgs/Image image
    sensor_msgs/RegionOfInterest roi
    ---
    Detections detections
    sensor_msgs/Image result_image
    ```

#### Parameters

Change in `ros/object_detector_tensorflow/config/params.yaml`

```bash
saved_model_path: "data/saved_model"  # Path to TensorFlow saved model folder
label_map_path: "data/label_map.txt"  # Text file with class names (one label per line)
min_probability: 0.5            # Minimum probability for detections to be reported
max_gpu_memory_fraction: 1.0    # Limits the GPU memory usage of the TensorFlow model to only a fraction (between 0 and 1)
result_image_size: [640,480]    # Dimensions of the result image [x,y]
```

## Dependencies

- tensorflow
- opencv

## How to install

    pip3 install tensorflow
    sudo apt install opencv
    pip3 install numpy
    sudo apt install ros-dashing-diagnostic-updater
    sudo apt install ros-dashing-ros1-bridge


## How to launch:

    ros2 launch object_detector_tensorflow detection.launch.py
    ros2 launch object_detector_tensorflow continuous_detection.launch.py

## Debugging

### generate a test image

    from cv_bridge import CvBridge, CvBridgeError
    import numpy as np
    bridge = CvBridge()
    np.zeros([960, 1280, 3], dtype=np.uint8)
    image = bridge.cv2_to_imgmsg(
        np.zeros([960, 1280, 3], dtype=np.uint8), encoding="bgr8")

### display test image from ROS-msgs

    image = bridge.imgmsg_to_cv2(image)
    cv2.imshow(image)
    cv2.waitKey(0)

### load image file

    # Test image from folder
    from cv_bridge import CvBridge
    import numpy as np
    import cv2

    bridge = CvBridge()

    img = cv2.imread('../data/test_image_0.png', 0) 
    image = bridge.cv2_to_imgmsg(img, encoding="bgr8")
