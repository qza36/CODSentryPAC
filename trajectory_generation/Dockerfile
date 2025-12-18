FROM ros:humble-perception-jammy

RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    cmake \
    libpcl-dev \
    ros-humble-pcl-msgs \
    ros-humble-gazebo-msgs \
    && rm -rf /var/lib/apt/lists/*

ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV ROS_DISTRO=humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages

LABEL description="planningENV" version="1.0" owner="ZhiangQi"