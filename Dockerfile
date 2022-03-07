##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=foxy
FROM ros:$ROS_DISTRO-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN rosdep update --rosdistro $ROS_DISTRO

# Update packages only if necessary, ~250MB
# RUN apt update && apt -y dist-upgrade

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-cv-bridge \
    python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# New versions necessary to prevent "skbuild" error from scikit-build
RUN python3 -m pip install -U pip setuptools
RUN pip3 install opencv-python
RUN pip3 install tensorflow

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=docker
ARG PASSWORD=petra
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=0
ENV UID=$UID
ENV GID=$GID
ENV USER=$USER
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=$DOMAIN_ID" >> /etc/bash.bashrc

USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                                 User Dependecies                         ##
##############################################################################
WORKDIR /home/$USER/ros2_ws/src
RUN git clone --depth 1 -b master https://project_107_bot:glpat-4sey2MxzfJ4xpykyZx59@www.w.hs-karlsruhe.de/gitlab/iras/common/object_detector_tensorflow.git

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

CMD ["ros2", "launch", "object_detector_tensorflow", "continuous_detection.launch.py"]