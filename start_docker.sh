#!/bin/sh
docker build -t object_detector_tensorflow/ros:dashing .
echo "Run Container"
docker run --name object_detector -it --net host --rm object_detector_tensorflow/ros:dashing
