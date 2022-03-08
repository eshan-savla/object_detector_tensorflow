#!/bin/sh
echo "Building object_detector_tensorflow/ros"

docker build -t object_detector_tensorflow/ros:foxy .

MOUNT_PATH=REPLACE_WITH_PATH_TO_CNN

echo "Running object_detector_tensorflow/ros with CNN from $MOUNT_PATH"

docker run -it \
--rm \
--gpus all \
--net host \
--name odtf \
-v $MOUNT_PATH:/home/docker/ros2_ws/src/object_detector_tensorflow/ros/data \
object_detector_tensorflow/ros:foxy
