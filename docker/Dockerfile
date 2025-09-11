# syntax=docker/dockerfile:1

############################
# Build args
############################
ARG UBUNTU_VERSION=22.04
ARG ROS_DISTRO=humble
ARG SIM=gazebo
# if isaac，BASE_IMAGE=nvcr.io/nvidia/isaac-sim:2023.1.1
ARG BASE_IMAGE=ubuntu:${UBUNTU_VERSION}

FROM ${BASE_IMAGE} AS base

# Re-declare ARGs in this stage and export to ENV
ARG UBUNTU_VERSION
ARG ROS_DISTRO
ARG SIM
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV SIM=${SIM}

############################
# basic dependencies
############################
RUN apt-get update && apt-get install -y \
    locales sudo software-properties-common \
    curl wget git gnupg2 lsb-release \
    python3-pip python3-venv apt-transport-https \
    net-tools iproute2 \
 && locale-gen en_US.UTF-8

############################
# ROS 2 + launch
############################
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-launch \
    ros-${ROS_DISTRO}-launch-ros \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

############################
# GUI: Xfce + TigerVNC + noVNC
############################
RUN apt-get update && apt-get install -y \
    curl wget gnupg2 lsb-release \
    git sudo nano vim bash-completion \
    net-tools iputils-ping \
    supervisor \
    # GUI + VNC + noVNC
    tigervnc-standalone-server tigervnc-common \
    xfce4 xfce4-terminal dbus-x11 x11-xserver-utils \
    novnc websockify \
    # For ROS2/Gazebo
    locales gnupg2 \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:1
# Consider passing this at runtime instead of baking into image:
# docker run -e VNC_PASSWORD=...
ENV VNC_PASSWORD=tennisbuddy
EXPOSE 5901 6080

# install desktop in vnc
RUN apt-get update && apt-get install -y --no-install-recommends \
    openbox xterm x11-apps \
    mesa-utils libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

############################
# Gazebo / Isaac dependencies
############################
# Sim selector (no tricky quoting needed)
RUN if [ "$SIM" = "gazebo" ]; then \
      apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-ros-gz \
        && rm -rf /var/lib/apt/lists/* ; \
    fi

RUN if [ "$SIM" = "isaac" ]; then \
      echo "[INFO] Expect BASE_IMAGE=nvcr.io/nvidia/isaac-sim:<tag>, and run with --gpus all"; \
    fi

############################
# User
############################
ENV USER=developer
RUN useradd -ms /bin/bash $USER && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

WORKDIR /workspace/tennisbuddy
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY scripts/start-vnc.sh /usr/local/bin/start-vnc.sh
# COPY scripts/start-gazebo.sh /workspace/start-gazebo.sh
COPY docker/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

COPY ..
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/start-vnc.sh

USER $USER
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]