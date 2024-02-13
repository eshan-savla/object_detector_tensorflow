#!/bin/sh

uid=$(eval "id -u")
gid=$(eval "id -g")

docker build --build-arg UID="$uid" --build-arg GID="$gid" -t object_detector_tensorflow/ros:foxy .