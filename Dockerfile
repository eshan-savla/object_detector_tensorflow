##############################################################################
##                                Base Image                                ##
##############################################################################
FROM rwthika/ros2-tf:humble-perception-tf2.11.0-py

##############################################################################
##                        Install ROS2 & dependencies                       ##
##############################################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-rqt* \
    ros-${ROS_DISTRO}-rc-common-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-rc-genicam-api \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && apt-get clean && rm -rf /var/lib/apt/lists/*rm

RUN python3 -m pip install -U pip
RUN pip3 install -U \
    setuptools \
    opencv-python

##############################################################################
##                                Create User                               ##
##############################################################################
ARG USER=docker
ARG PASSWORD=docker
ARG UID=1002
ARG GID=1002
ENV UID=$UID
ENV GID=$GID
ENV USER=$USER
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
# RUN git clone --depth 1 https://github.com/roboception/rc_genicam_driver_ros2.git
COPY . ./object_detector_tensorflow

##############################################################################
##                             Build ROS and run                            ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
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

# CMD ["ros2", "launch", "object_detector_tensorflow", "continuous_detection.launch.py"]
# CMD ["ros2", "run", "rc_genicam_driver", "rc_genicam_driver", "--ros-args", "-r", "/stereo/left/image_rect_color:=/image"]
CMD /bin/bash
