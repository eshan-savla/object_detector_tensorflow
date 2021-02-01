# Object Detector Tensorflow

Python based ROS Node for TensorFlow Object Detection Inference

## How to run:

The parameters for each node can be changed in `config/param.yaml`

To start the nodes with the custom parameters use the launch script:

`ros2 launch object_detector_tensorflow object_detector_tensorflow.launch.py `

Or give the path to the param file as command argument:

`ros2 run object_detector_tensorflow continuous_detection_node __params:=/home/andreas/petra_ws/src/object_detector_tensorflow/config/params.yaml`

---

`ros2 param list`

`ros2 param set /continuous_detection_node min_probability 0.5`

