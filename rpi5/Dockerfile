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
    ros-jazzy-compressed-image-transport \
    ros-jazzy-v4l2-camera \
    nodejs \
    npm \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-serial

RUN apt update && apt install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  v4l-utils && \
  rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash

# 🔹 Prompt chiaro per il container (valido per tutte le shell login/interattive)
SHELL ["/bin/bash", "-c"]
RUN echo 'export PS1="[ROS2-CONTAINER jazzy] \\u@\\h:\\w$ "' > /etc/profile.d/ros2_prompt.sh
RUN echo "source /opt/ros/jazzy/setup.bash" > /etc/profile.d/ros2_setup.sh
RUN chmod +x /etc/profile.d/ros2_setup.sh

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

