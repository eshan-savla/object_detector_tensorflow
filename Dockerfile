# Run build with --build-arg TF_RELEASE="X.X.X-X" for other releases than 2.3.0-gpu
ARG TF_RELEASE=2.3.0-gpu
FROM tensorflow/tensorflow:1.13.2$TF_RELEASE
ARG DEBIAN_FRONTEND=noninteractive

#### ROS 2 Installation ####

# Run build with --build-arg ROS_DISTRO="X" for other releases than dashing
ARG ROS_DISTRO=dashing

# Setup Locale
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Setup Sources
RUN sudo apt update && sudo apt install curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 packages
RUN sudo apt update
RUN sudo apt install ros-$ROS_DISTRO-desktop

# Environment setup
RUN source /opt/ros/crystal/setup.bash

# Install argcomplete
RUN sudo apt install python3-argcomplete

# setup entrypoint
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

############################

WORKDIR /workspace

# set up the ros node
COPY object_detection_tensorflow ./object_detection_tensorflow

# deploy the default CNN
COPY data/* /

ENV PYTHONUNBUFFERED 1
CMD [ "python", "-m", "object_detection_tensorflow" ]

STOPSIGNAL SIGINT
