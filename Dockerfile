FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Chicago

RUN cd $HOME
RUN apt update && apt upgrade -y
RUN apt install tzdata -y && apt install -y sudo

# Change shell to bash and add user
SHELL ["/bin/bash", "-c"]
ARG USER=myuser
ARG HOME=/home/myuser
RUN useradd -ms /bin/bash $USER
RUN echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USER
WORKDIR $HOME

# Set the locale
RUN sudo sudo apt update && sudo sudo apt install locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Install software-properties-common and add repository
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe

# Add key and repository for ROS2
RUN sudo apt update && sudo apt install curl -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install and Source ROS2
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install ros-humble-desktop -y
RUN sudo apt install ros-humble-ros-base -y
RUN sudo apt install ros-dev-tools -y
RUN source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

# Install Gazebo
RUN sudo apt update && sudo apt install curl lsb-release gnupg && \
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt update && sudo apt install gz-harmonic -y

# Install humble-gz-bridge
RUN sudo apt update && sudo apt install ros-humble-ros-gzharmonic -y && \
    export GZ_VERSION=harmonic && echo "export GZ_VERSION=harmonic" >> $HOME/.bashrc

# Install dependencies
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install -y build-essential \
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

RUN source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc && \
    export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models && echo "export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models" >> $HOME/.bashrc

RUN sudo apt update && sudo apt -y upgrade && sudo apt -y install mesa-utils

ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
ENV LIBVA_DRIVER_NAME=d3d12

RUN export MESA_D3D12_DEFAULT_NAME=NVIDIA && echo "export MESA_D3D12_DEFAULT_NAME=NVIDIA" >> $HOME/.bashrc

# Create workspace for package and ardupilot
RUN cd $HOME && mkdir -p $HOME/ros2_ws/src && sudo rosdep init

# Install ardupilot
RUN cd $HOME && \
    sudo apt install gitk git-gui -y && \
    sudo apt install gcc-arm-none-eabi -y && \
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

RUN cd $HOME/ardupilot && \
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN . $HOME/.profile

# Configure waf and add important paths to bashrc
RUN cd $HOME/ardupilot && \
    ./waf distclean && \
    ./waf configure --board=sitl && \
    ./waf copter

RUN export PATH=\$PATH:$HOME/ardupilot/Tools/autotest && \
    echo "export PATH=\$PATH:$HOME/ardupilot/Tools/autotest" >> $HOME/.bashrc && \
    export PATH=/usr/lib/ccache:$PATH && \
    echo "export PATH=/usr/lib/ccache:$PATH" >> $HOME/.bashrc

# Install dependency Micro-XRCE-DDS
RUN sudo apt install default-jre -y && \
    sudo apt install openjdk-17-jdk -y && \
    export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 && \
    echo 'export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64' >> $HOME/.bashrc && \
    cd $HOME && \
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git && \
    cd Micro-XRCE-DDS-Gen && \
    ./gradlew assemble && \
    export PATH=\$PATH:$PWD/scripts && \
    echo "export PATH=\$PATH:$PWD/scripts" >> $HOME/.bashrc && \
    export PATH=$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts && \
    echo "export PATH=$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts" >> $HOME/.bashrc

# Install ardupilot and micro-ros-agent into your workspace
RUN cd $HOME/ros2_ws && \
    sudo apt install python3-vcstool -y && \
    vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src && \
    sudo apt update && source /opt/ros/humble/setup.bash && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    export PATH=$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts && \
    colcon build --packages-up-to ardupilot_dds_tests

# Install ardupilot_gazebo plugin into workspace
RUN cd $HOME/ros2_ws/src && \
    sudo apt update && \
    sudo apt install -y libgz-sim8-dev rapidjson-dev && \
    sudo apt install -y libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl && \
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git && \
    export GZ_VERSION=harmonic && \
    cd ardupilot_gazebo && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j4 && \
    echo "export GZ_VERSION=harmonic" >> $HOME/.bashrc && \
    sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list' && \
    cd $HOME/ros2_ws && rosdep update && \
    rosdep resolve gz-harmonic --rosdistro humble && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble

RUN export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH" >> $HOME/.bashrc && \
    export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH && \
    echo "export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH" >> $HOME/.bashrc

RUN cd $HOME/ros2_ws/src && \
    git clone https://github.com/ArduPilot/SITL_Models.git

RUN cd $HOME/ros2_ws/src && \
    git clone https://github.com/ArduPilot/ardupilot_gz.git && \
    export GZ_VERSION=harmonic && \
    source /opt/ros/humble/setup.bash && cd $HOME/ros2_ws && \
    sudo apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

RUN cd $HOME/ros2_ws/src && \
    git clone https://github.com/gazebosim/ros_gz.git -b humble && \
    cd $HOME/ros2_ws && \
    rosdep install -r --from-paths src -i -y --rosdistro humble && \
    source /opt/ros/humble/setup.bash && cd $HOME/ros2_ws && \
    export ROS_DISTRO=humble && \
    echo "export ROS_DISTRO=humble" >> $HOME/.bashrc && \
    export PATH=$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts && \
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH && \
    export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH && \
    export GZ_VERSION=harmonic && \
    colcon build

RUN cd $HOME/ros2_ws/src && \
    mkdir multi_robot_follow

VOLUME [ "$HOME/ros2_ws/src/multi_robot_follow" ]