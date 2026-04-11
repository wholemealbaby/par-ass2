# /docker/Dockerfile

########################
# Base for ROS install
ARG ROS_DISTRO=jazzy
FROM husarion/rosbot:jazzy-0.16.2-20251106 AS aiil_rosbase

SHELL ["/bin/bash", "-c"]

# Install System Utilities
RUN apt update && apt install -y \
    less \
    vim \
    ros-dev-tools \
    && apt clean && rm -rf /var/lib/apt/lists/*

# Install ROS2 Packages (Including your requested tf_transformations)
RUN apt update && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    apt install -y \
        ros-$ROS_DISTRO-find-object-2d \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-nav2-common \
        ros-$ROS_DISTRO-slam-toolbox \
        ros-$ROS_DISTRO-depthai-ros \
        # Transformations dependencies
        ros-$ROS_DISTRO-tf-transformations \
        python3-transforms3d \
    && apt clean && rm -rf /var/lib/apt/lists/*

########################
# Setup for workspace
# Correcting ARG to jazzy for consistency
ARG ROS_DISTRO=jazzy
FROM aiil_rosbase

WORKDIR /ros2_ws
RUN rm -rf *

# We assume the entrypoint exists in the base image or robot system. 
# If you have a custom one in your repo, uncomment the next lines:
# COPY ./docker/ros_entrypoint.sh /
# RUN chmod +rwx /ros_entrypoint.sh
# ENTRYPOINT ["/ros_entrypoint.sh"]