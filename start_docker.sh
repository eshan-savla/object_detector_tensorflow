#!/bin/sh

xhost + local:root

docker run \
    -it \
    --privileged \
    --net host \
    -e DISPLAY=$DISPLAY \
    --env-file .env \
    --rm \
    --gpus all \
    --name odtf \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $PWD/ros/object_detector_tensorflow/data:/home/docker/ros2_ws/src/object_detector_tensorflow/ros/object_detector_tensorflow/data \
    object_detector_tensorflow/ros:foxy
    