# Object Detector Tensorflow

Python based ROS Node for TensorFlow Object Detection Inference

## Install requirements

    pip3 install tensorflow
    sudo apt install opencv
    pip3 install numpy
    sudo apt install ros-dashing-diagnostic-updater
    sudo apt install ros-dashing-ros1-bridge


## How to run:

The parameters for each node can be changed in `config/params.yaml`

To start the nodes with the custom parameters use the launch script:

`ros2 launch object_detector_tensorflow object_detector_tensorflow.launch.py `

Or give the path to the param file as command argument:

`ros2 run object_detector_tensorflow continuous_detection_node __params:=/home/andreas/petra_ws/src/object_detector_tensorflow/config/params.yaml`

---

`ros2 param list`

`ros2 param set /continuous_detection_node min_probability 0.5`

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
