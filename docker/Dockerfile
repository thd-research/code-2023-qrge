ARG from
FROM ${from}

ENV LANG C.UTF-8

# Timezone Configuration
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND = noninteractive

ENV CXX=/usr/bin/g++-8
ENV CC=/usr/bin/gcc-8
ENV WORKSPACE=/raisim_workspace
ENV LOCAL_BUILD=/raisim_build
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${LOCAL_BUILD}/lib
ENV PYTHONPATH=$PYTHONPATH:${LOCAL_BUILD}/lib

RUN mkdir .raisim

RUN apt-get update && apt-get install -y git software-properties-common gcc-8 g++-8

RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y firefox

# ==================================================================
# display
# ------------------------------------------------------------------
# For nvidia GUI issues
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64
EXPOSE 6006

RUN add-apt-repository -y -r ppa:graphics-drivers/ppa && \
	apt update && \
	apt upgrade -y

RUN apt-get update && apt install -y nvidia-driver-525 nvidia-settings vulkan-tools vulkan-utils
# RUN apt-get install -y ubuntu-drivers-common

# ==================================================================

## GUI tunnel from docker
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

RUN apt-get update && apt-get install -y --no-install-recommends \
        libglvnd0 libglvnd0:i386 \
        libgl1 libgl1:i386 \
        libglx0 libglx0:i386 \
        libegl1 libegl1:i386 \
        libgles2 libgles2:i386 && \
    rm -rf /var/lib/apt/lists/*

# COPY ~/home/roman/github/raisim_a1/docker/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

# ==================================================================

# RUN apt-get update && apt-get install -y python3.7
# RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1
# RUN rm -f /usr/bin/python3 && ln -s /usr/bin/python3 /usr/bin/python3.7

RUN apt-get update && apt-get install -y gedit nano minizip ffmpeg libgles2-mesa-dev libxt-dev libxaw7-dev libsdl2-dev libzzip-dev libfreeimage-dev libfreetype6-dev libpugixml-dev

RUN add-apt-repository ppa:deadsnakes/ppa && apt-get update && apt-get install -y cmake python3.6-dev python3-pip libpython3.6-dev libeigen3-dev




#RUN pip3 install torch==1.8.1+cu111 -f https://download.pytorch.org/whl/torch_stable.html
RUN pip3 install torch -f https://download.pytorch.org/whl/torch_stable.html

#RUN pip3 install tensorflow setuptools tensorboard==2.4.1 ruamel.yaml numpy==1.19.5
RUN pip3 install tensorflow setuptools tensorboard ruamel.yaml numpy==1.19.5


RUN apt-get update && apt install -y nvidia-driver-460 nvidia-settings vulkan-tools vulkan-utils

RUN echo "alias file='cd /raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_anymal'" >> ~/.bashrc
# RUN add-apt-repository -y ppa:oibaf/graphics-drivers && apt update && apt #upgrade -y && apt install -y libvulkan1 mesa-vulkan-drivers vulkan-utils

