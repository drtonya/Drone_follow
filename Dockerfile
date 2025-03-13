FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Chicago

RUN cd /root
RUN apt update && apt upgrade -y
RUN apt install tzdata -y

# Change shell to bash
SHELL ["/bin/bash", "-c"] 


# Set the locale
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Install software-properties-common and add repository
RUN apt install software-properties-common -y
RUN add-apt-repository universe

# Add key and repository for ROS2
RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install and Source ROS2
RUN apt update && apt upgrade -y
RUN apt install ros-humble-desktop-full -y
RUN apt install ros-humble-ros-base -y
RUN apt install ros-dev-tools -y
RUN source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

# Install Gazebo
# RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# RUN apt update && apt install gz-garden -y

# Install dependencies
RUN apt update && apt upgrade -y
RUN apt install -y build-essential \
    cmake \
    python3 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-pip \
    python3-venv \
    clang \
    lldb \
    ninja-build \
    libgtest-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    sudo \
    wget \
    tmux \
    ruby \
    tmuxinator

# Create virtual environment and setuptools version is important
RUN python3 -m venv /root/px4-venv
RUN source /root/px4-venv/bin/activate && pip3 install -U empy pyros-genmsg setuptools==65.5.0

# Setup Micro XRCE-DDS
RUN cd /root && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make && make install && ldconfig /usr/local/lib/

RUN apt install -y apt-utils
RUN python3 -m pip install --upgrade pip

# Install PX4
RUN cd /root && \
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive && source /root/px4-venv/bin/activate && \
    bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh && \
    cd PX4-Autopilot && \
    make px4_sitl

# Build ROS2 workspaces
RUN mkdir -p /root/ws_sensor_combined/src/ && \
    cd /root/ws_sensor_combined/src/ && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    cd /root/ws_sensor_combined && \
    source /opt/ros/humble/setup.bash && \
    colcon build

RUN mkdir -p /root/ws_offboard_control/src/ && \
    cd /root/ws_offboard_control/src/ && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    cd /root/ws_offboard_control && \
    source /opt/ros/humble/setup.bash && \
    colcon build

# Install python requirements
RUN source /root/px4-venv/bin/activate && pip3 install mavsdk \
    aioconsole \
    pygame \
    numpy \
    opencv-python \
    ultralytics

RUN apt install ros-humble-ros-gzgarden -y

# Uninstall because of version mismatch
RUN source /root/px4-venv/bin/activate && pip3 uninstall -y numpy && pip3 install numpy==1.26.4

# Copy models and worlds from local repository
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY PX4-ROS2-Gazebo-YOLOv8 /root/PX4-ROS2-Gazebo-YOLOv8/
COPY PX4-ROS2-Gazebo-YOLOv8/models/. /root/.gz/models/
COPY PX4-ROS2-Gazebo-YOLOv8/models_docker/. /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY PX4-ROS2-Gazebo-YOLOv8/worlds/default_docker.sdf /root/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

# Modify camera angle
RUN sed -i 's|<pose>.12 .03 .242 0 0 0</pose>|<pose>.15 .029 .21 0 0.7854 0</pose>|' /root/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf

# Bash
RUN source /root/ws_sensor_combined/install/setup.bash && echo "source /root/ws_sensor_combined/install/setup.bash" >> /root/.bashrc && \
    source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    export GZ_SIM_RESOURCE_PATH=/root/.gz/models && echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models" >> /root/.bashrc

RUN apt update && apt -y upgrade && apt -y install mesa-utils

ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
ENV LIBVA_DRIVER_NAME=d3d12

RUN export MESA_D3D12_DEFAULT_NAME=NVIDIA && echo "export MESA_D3D12_DEFAULT_NAME=NVIDIA" >> /root/.bashrc