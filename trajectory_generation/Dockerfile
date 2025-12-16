FROM osrf/ros:humble-desktop

RUN apt update && apt install ros-humble-pcl* -y && apt install ros-humble-gazebo-msgs -y
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV ROS_DISTRO=humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages

LABEL description="planningENV" version="1.0" owner="ZhiangQi"