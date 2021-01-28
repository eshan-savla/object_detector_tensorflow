# Object Detector Tensorflow

Python based ROS Node for TensorFlow Object Detection Inference

`ros2 run object_detector_tensorflow continuous_detection_node __params:=/home/andreas/petra_ws/src/object_detector_tensorflow/config/params.yaml`

`ros2 param list`

`ros2 param set /continuous_detection_node min_probability 0.5`

`ros2 launch object_detector_tensorflow object_detector_tensorflow.launch.py `