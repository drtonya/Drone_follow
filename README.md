# Running the docker container with WSL2

### Useful info on setting up docker with wsl:
- https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers

### Use this command when using the docker run command:
```
sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
    -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY \
    --device /dev/dri/card0 --device /dev/dri/renderD128 \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e PULSE_SERVER=$PULSE_SERVER --gpus all <image name>
```
- These are necessary to let the container know:
    - What device and display you are using
    - Make the container use your GPU
    - Starting an X-11 server
    - Giving the container necessary info from wsl

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