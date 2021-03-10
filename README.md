# Object Detector Tensorflow

Python based ROS Node for TensorFlow Object Detection Inference

## Install requirements

    pip3 install tensorflow
    sudo apt install opencv
    pip3 install numpy
    sudo apt install ros-dashing-diagnostic-updater
    sudo apt install ros-dashing-ros1-bridge


## How to run:

The parameters for each node can be changed in `config/param.yaml`

To start the nodes with the custom parameters use the launch script:

    ros2 launch object_detector_tensorflow detection.launch.py
    ros2 launch object_detector_tensorflow continuous_detection.launch.py

Or give the path to the param file as command argument:

    ros2 run object_detector_tensorflow continuous_detection_node.py __params:=/home/andreas/Code/iris_ws/install/object_detector_tensorflow/../../src/object_detector_tensorflow/ros/config/params.yaml

---

    ros2 param list
    ros2 param set /continuous_detection_node min_probability 0.5

### generate a test image

    from cv_bridge import CvBridge, CvBridgeError
        import numpy as np
        bridge = CvBridge()
        np.zeros([960, 1280, 3], dtype=np.uint8)
        image = bridge.cv2_to_imgmsg(
            np.zeros([960, 1280, 3], dtype=np.uint8), encoding="bgr8")

### display test image from ROS-msgs

    image = self.bridge.imgmsg_to_cv2(image)
    cv2.imshow()
    cv2.waitKey(0)

export PYTHONPATH=$PYTHONPATH:~/Code/iris_ws/src/object_detector_tensorflow
