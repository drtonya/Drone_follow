# Running the docker container with WSL2

### Useful info on setting up docker with wsl:
- https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers

### Use this command when using the docker run command:
```
sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl \
    -v /home/tonya/dock_ock/multi_robot_follow:/home/myuser/ros2_ws/src/multi_robot_follow \
    --device=/dev/dxg -e DISPLAY=$DISPLAY \
    --device /dev/dri/card0 --device /dev/dri/renderD128 \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e PULSE_SERVER=$PULSE_SERVER --gpus all <image-name>
```
- These are necessary to let the container know:
    - What device and display you are using
    - Make the container use your GPU
    - Starting an X-11 server
    - Giving the container necessary info from wsl

- After running, you might have to do some of these:
    - source /home/myuser/.bashrc
    - might have to build workspace again (MAKE SURE NOT IN SRC)
    - might have to do ./gradlew assemble in Micro DDS folder again
    - source /home/myuser/.profile
    - export PATH=$PATH:/home/myuser/ros2_ws/src/ardupilot/Tools/autotest
    - export PATH=/usr/lib/ccache:$PATH
    - export GZ_VERSION=harmonic
    - export PATH=$PATH:/home/myuser/ros2_ws/Micro-XRCE-DDS-Gen/scripts

- More information on running containers with wsl here:
    - https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md

### If it does not work immediately after, try these commands while in the container:
```
export LD_LIBRARY_PATH=/usr/lib/wsl/lib
export LIBVA_DRIVER_NAME=d3d12
export MESA_D3D12_DEFAULT_NAME=NVIDIA
```
- Other possible fixes:

    - https://github.com/microsoft/WSL/issues/7507


# Alternative for using PX4

```
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
RUN source /root/ws_sensor_combined/install/setup.bash && echo "source /root/ws_sensor_combined/install/setup.bash" >> /root/.bashrc





# Install ardupilot and micro-ros-agent into your workspace
RUN cd $HOME/ros2_ws && \
    sudo apt install python3-vcstool -y && \
    vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src && \
    sudo apt update && source /opt/ros/humble/setup.bash && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    export PATH=$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts && \
    colcon build --packages-up-to ardupilot_dds_tests

RUN cd $HOME/ros2_ws/src && \
    git clone https://github.com/ArduPilot/SITL_Models.git && \
    export GZ_VERSION=harmonic && \
    colcon build

# Install humble-gz-bridge
RUN sudo apt update && sudo apt install ros-humble-ros-gzharmonic -y && \
    export GZ_VERSION=harmonic && echo "export GZ_VERSION=harmonic" >> ${HOME_DIR}/.bashrc

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
    clang \
    lldb \
    ninja-build \
    libgtest-dev \
    libeigen3-dev \
    libyaml-dev \
    sudo \
    wget \
```