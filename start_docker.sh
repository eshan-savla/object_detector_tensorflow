#!/bin/sh

xhost + local:root

if [ "$1" = "--no-gpus" ]; then
    docker run \
        -it \
        --privileged \
        --net host \
        -e DISPLAY=$DISPLAY \
        --env-file .env \
        --rm \
        --name odtf \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $PWD/:/home/docker/ros2_ws/src/object_detector_tensorflow/ \
        -v $PWD/.vscode:/home/docker/ros2_ws/src/.vscode \
        object_detector_tensorflow/ros:humble
else
    docker run \
        -it \
        --privileged \
        --net host \
        -e DISPLAY=$DISPLAY \
        --env-file .env \
        --rm \
        --name odtf \
        --gpus all \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $PWD/:/home/docker/ros2_ws/src/object_detector_tensorflow/ \
        -v $PWD/.vscode:/home/docker/ros2_ws/src/.vscode \
        object_detector_tensorflow/ros:humble
fi
    # -v $PWD/ros/object_detector_tensorflow/data:/home/docker/ros2_ws/src/object_detector_tensorflow/ros/object_detector_tensorflow/data \
    