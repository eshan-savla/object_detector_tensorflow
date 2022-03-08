##############################################################################
##                                Base Image                                ##
##############################################################################
ARG TF_VERSION=2.8.0
FROM tensorflow/tensorflow:$TF_VERSION-gpu

RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    git \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm

##############################################################################
##                        Install ROS2 & dependencies                       ##
##############################################################################
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

ARG ROS_DISTRO=foxy
RUN curl https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-diagnostic-updater \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*rm

RUN source /opt/ros/$ROS_DISTRO/setup.bash

ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN rosdep init
RUN rosdep update --rosdistro $ROS_DISTRO

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U pip
RUN pip3 install -U \
    setuptools \
    opencv-python

##############################################################################
##                                Create User                               ##
##############################################################################
ARG USER=docker
ARG PASSWORD=petra
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=0
ENV UID=$UID
ENV GID=$GID
ENV USER=$USER
ENV ROS_DOMAIN_ID=$DOMAIN_ID
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                             User Dependecies                             ##
##############################################################################
WORKDIR /home/$USER/ros2_ws/src
RUN git clone --depth 1 -b master https://project_107_bot:glpat-4sey2MxzfJ4xpykyZx59@www.w.hs-karlsruhe.de/gitlab/iras/common/object_detector_tensorflow.git

##############################################################################
##                             Build ROS and run                            ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc

RUN touch ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh
RUN echo "#!/bin/bash" >> ros_entrypoint.sh
RUN echo "set -e" >> ros_entrypoint.sh
RUN echo "# setup ros environment" >> ros_entrypoint.sh
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ros_entrypoint.sh
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> ros_entrypoint.sh
RUN echo "exec \$@" >> ros_entrypoint.sh
RUN sudo mv ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "object_detector_tensorflow", "continuous_detection.launch.py"]
