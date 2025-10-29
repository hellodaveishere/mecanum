FROM osrf/ros:jazzy-desktop-noble
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -d /home/ws $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


RUN usermod -a -G dialout $USERNAME

RUN apt-get update && apt-get install -y \
    nlohmann-json3-dev ros-jazzy-serial-driver python3-serial \
    ros-jazzy-asio-cmake-module libserial-dev \
    ros-jazzy-diagnostic-updater libboost-all-dev\
    ros-jazzy-hardware-interface \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-imu-tools \
    ros-jazzy-robot-localization \
    ros-jazzy-rosbridge-server \
    ros-jazzy-battery-state-broadcaster \
    ros-jazzy-topic-tools \
    nodejs \
    npm \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-serial

# Ref. https://github.com/RozaGkliva/ros2_serial
# 
#install serial library for c++
#git clone https://github.com/RoverRobotics-forks/serial-ros2
#cd serial-ros2/
#make
#make install
#cd ~/dev_ws/ && colcon build --packages-select serial

# Installa pyserial via pip
#RUN pip3 install pyserial

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]