#!/bin/sh
echo "Building object_detector_tensorflow/ros"

#uid=$(eval "id -u")
#gid=$(eval "id -g")
#--build-arg UID="$uid" --build-arg GID="$gid"
docker build -t object_detector_tensorflow/ros:foxy .