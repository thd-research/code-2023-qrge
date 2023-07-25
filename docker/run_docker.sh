#!/bin/bash

xhost +local:docker || true

if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker run --gpus all \
                -ti --rm \
                -e "DISPLAY" \
                -e "QT_X11_NO_MITSHM=1" \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -e XAUTHORITY \
                -v /dev:/dev \
                -v /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace:/raisim_workspace \
                -v /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_build:/raisim_build \
                -v /home/$USER/.raisim:/root/.raisim \
               --net=host \
               --privileged \
               --name raisim raisim-img

else

    echo "[!] If you wanna use nvidia gpu, please use script with -n or --nvidia argument"
    docker run  -ti --rm \
                -e "DISPLAY" \
                -e "QT_X11_NO_MITSHM=1" \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -e XAUTHORITY \
                -v /dev:/dev \
                -v /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace:/raisim_workspace \
                -v /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_build:/raisim_build \
                -v /home/$USER/.raisim:/root/.raisim \
               --net=host \
               --privileged \
               --name raisim raisim-img
fi
