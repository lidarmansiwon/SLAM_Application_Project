# ========================
# 1. Base image & setup
# ========================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1.1 기본 도구 설치
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales sudo wget git software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# 1.2 로케일 설정
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ========================
# 2. ROS 2 설치
# ========================
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros2-archive-keyring.gpg

RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

# ROS 환경변수 설정
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# ========================
# 3. ROS 2 및 시스템 의존성 설치
# ========================
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-rviz2 \
    build-essential \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    iputils-ping \
    net-tools

# ========================
# 4. Python requirements 설치 (옵션)
# ========================
#COPY requirements.txt /tmp/
#RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ========================
# 5. ROS 2 workspace 생성 및 ouster_ros clone
# ========================
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

# ouster_ros나 원하는 패키지들 clone (예시)
RUN git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

# ========================
# 6. colcon build
# ========================
WORKDIR /root/ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# RUN python3 -m pip install ~root/SLAM_Application_Project/dependency/flask-3.1.1.tar.gz


# ========================
# 7. Default 실행 셸
# ========================
CMD ["bash"]

