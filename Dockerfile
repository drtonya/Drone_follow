FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Chicago

RUN cd ${HOME_DIR}
RUN apt update && apt upgrade -y
RUN apt install tzdata -y && apt install -y sudo

# Change shell to bash and add user
SHELL ["/bin/bash", "-c"]
ARG USER_UID=1000
ARG USER_GID=1000
ARG USER_NAME=myuser
ARG HOME_DIR=/home/myuser
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash
RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ${USER_NAME}
WORKDIR ${HOME_DIR}

# Install Mesa-utils and set environment variables for GPU acceleration
RUN sudo apt update && sudo apt -y upgrade && sudo apt -y install mesa-utils
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
ENV LIBVA_DRIVER_NAME=d3d12
RUN export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json && echo "export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json" >> ${HOME_DIR}/.bashrc
RUN export MESA_D3D12_DEFAULT_NAME=NVIDIA && echo "export MESA_D3D12_DEFAULT_NAME=NVIDIA" >> ${HOME_DIR}/.bashrc

# Install ardupilot
## Create workspace for package and ardupilot


# Install ROS2
## Set the locale
RUN sudo sudo apt update && sudo sudo apt install locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

## Install software-properties-common and add repository
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe

## Add key and repository for ROS2
RUN sudo apt update && sudo apt install curl -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install and Source ROS2
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install ros-humble-desktop -y
RUN sudo apt install ros-humble-ros-base -y
RUN sudo apt install ros-dev-tools -y
RUN source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

# Install Ardupilot ROS2 dependencies
## Create workspace
RUN cd ${HOME_DIR} && mkdir -p ${HOME_DIR}/ros2_ws/src

## Install dependency Micro-XRCE-DDS
RUN sudo apt install default-jre -y && \
    cd ${HOME_DIR}/ros2_ws && \
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git && \
    cd Micro-XRCE-DDS-Gen && \
    ./gradlew assemble && \
    export PATH=\$PATH:$PWD/scripts && \
    echo "export PATH=\$PATH:$PWD/scripts" >> ${HOME_DIR}/.bashrc && \
    export PATH=$PATH:${HOME_DIR}/ros2_ws/Micro-XRCE-DDS-Gen/scripts && \
    echo "export PATH=$PATH:${HOME_DIR}/ros2_ws/Micro-XRCE-DDS-Gen/scripts" >> ${HOME_DIR}/.bashrc

## Install ardupilot and micro-ros-agent into your workspace
RUN cd ${HOME_DIR}/ros2_ws && \
    sudo apt install gitk git-gui -y && \
    sudo apt install gcc-arm-none-eabi -y && \
    sudo apt install python3-vcstool -y && \
    vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src && \
    sudo apt update && source /opt/ros/humble/setup.bash && \
    sudo rosdep init && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN cd ${HOME_DIR}/ros2_ws/src/ardupilot && \
    USER=myuser \
    HOME=/home/myuser \
    Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    . ${HOME_DIR}/.profile

## Configure waf and add important paths to bashrc
RUN cd ${HOME_DIR}/ros2_ws/src/ardupilot && \
    ./waf distclean && \
    ./waf configure --board=sitl && \
    ./waf copter

RUN export PATH=$PATH:${HOME_DIR}/ros2_ws/src/ardupilot/Tools/autotest && \
    echo "export PATH=$PATH:${HOME_DIR}/ros2_ws/src/ardupilot/Tools/autotest" >> ${HOME_DIR}/.bashrc && \
    export PATH=/usr/lib/ccache:$PATH && \
    echo "export PATH=/usr/lib/ccache:$PATH" >> ${HOME_DIR}/.bashrc
    
RUN sudo apt update && \
    cd ${HOME_DIR}/ros2_ws && \
    rosdep update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

## Test Build
RUN sudo apt update && sudo apt install -y build-essential \
    cmake \
    python3 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-pip \
    clang \
    lldb \
    ninja-build \
    libgtest-dev \
    libeigen3-dev \
    libyaml-dev \
    sudo \
    wget

RUN cd ${HOME_DIR}/ros2_ws && sudo apt update && source /opt/ros/humble/setup.bash && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . ${HOME_DIR}/.bashrc && export PATH=$PATH:${HOME_DIR}/ros2_ws/src/ardupilot/Tools/autotest && \
    export PATH=$PATH:${HOME_DIR}/ros2_ws/Micro-XRCE-DDS-Gen/scripts && \
    colcon build --packages-up-to ardupilot_dds_tests

# Install Gazebo
RUN sudo apt update && sudo apt install curl lsb-release gnupg && \
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt update && sudo apt install gz-harmonic -y

RUN export GZ_SIM_RESOURCE_PATH=${HOME_DIR}/.gz/models && echo "export GZ_SIM_RESOURCE_PATH=${HOME_DIR}/.gz/models" >> ${HOME_DIR}/.bashrc

# Install ardupilot_gz
RUN cd ${HOME_DIR}/ros2_ws && export GZ_SIM_RESOURCE_PATH=${HOME_DIR}/.gz/models && \
    source /opt/ros/humble/setup.bash && export PATH=$PATH:${HOME_DIR}/ros2_ws/Micro-XRCE-DDS-Gen/scripts && \
    export GZ_VERSION=harmonic && source ${HOME_DIR}/.bashrc && \
    vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src && \
    export GZ_VERSION=harmonic && echo "export GZ_VERSION=harmonic" >> ${HOME_DIR}/.bashrc

RUN sudo apt install wget && \
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt update && \
    sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list && \
    rosdep update

RUN export PATH=$PATH:${HOME_DIR}/ros2_ws/src/ardupilot/Tools/autotest && \
    echo "export PATH=$PATH:${HOME_DIR}/ros2_ws/src/ardupilot/Tools/autotest" >> ${HOME_DIR}/.bashrc && \
    export PATH=/usr/lib/ccache:$PATH && \
    echo "export PATH=/usr/lib/ccache:$PATH" >> ${HOME_DIR}/.bashrc

RUN cd ${HOME_DIR}/ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    source ${HOME_DIR}/.bashrc && \
    . ${HOME_DIR}/.bashrc && export PATH=$PATH:${HOME_DIR}/ros2_ws/src/ardupilot/Tools/autotest && \
    export PATH=$PATH:${HOME_DIR}/ros2_ws/Micro-XRCE-DDS-Gen/scripts && \
    sudo apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build

RUN cd ${HOME_DIR}/ros2_ws/src && \
    mkdir multi_robot_follow

VOLUME [ "${HOME_DIR}/ros2_ws/src/multi_robot_follow" ]
CMD ["bash"]