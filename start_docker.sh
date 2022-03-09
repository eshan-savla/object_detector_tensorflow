#!/bin/sh
echo "Building object_detector_tensorflow/ros"

#uid=$(eval "id -u")
#gid=$(eval "id -g")
#--build-arg UID="$uid" --build-arg GID="$gid"
docker build -t object_detector_tensorflow/ros:foxy .

MOUNT_PATH=/home/zaan0001/Code/aip/object_detector_tensorflow/ros/data

echo "Running object_detector_tensorflow/ros with CNN from $MOUNT_PATH"

xhost + local:root

docker run -it \
--privileged \
--net host \
-e DISPLAY=$DISPLAY \
--rm \
--gpus all \
--name odtf \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v $MOUNT_PATH:/home/docker/ros2_ws/src/object_detector_tensorflow/ros/data \
object_detector_tensorflow/ros:foxy
