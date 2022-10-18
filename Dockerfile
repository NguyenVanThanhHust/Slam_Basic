FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub

# Install some basic utilities
RUN apt-get update && apt-get install -y \
    curl \
    ca-certificates \
    sudo \
    git \
    bzip2 \
    libx11-6 \
    checkinstall \
    locales \
    lsb-release \
    mesa-utils \
    subversion \
    nano \
    terminator \
    xterm \
    wget \
    htop \
    libssl-dev \
    build-essential \
    dbus-x11 \
    software-properties-common \
    gdb valgrind \
    libeigen3-dev \
    libboost-all-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /git_installed_lib/

RUN git clone https://gitlab.kitware.com/cmake/cmake.git && \
cd cmake && git checkout tags/v3.20.2 && ./bootstrap --parallel=8 && make -j8 && make install && \
cd .. && rm -rf cmake

WORKDIR /git_installed_lib/

RUN git clone https://github.com/opencv/opencv.git -b 4.5.2
RUN git clone https://github.com/opencv/opencv_contrib.git -b 4.5.2

WORKDIR /git_installed_lib//opencv/build/
RUN cmake .. -D BUILD_opencv_java=OFF -D BUILD_opencv_python=0  -D BUILD_opencv_python2=0 -D BUILD_opencv_python3=0 -DOPENCV_EXTRA_MODULES_PATH= ../../opencv_contrib/modules/ -D OPENCV_ENABLE_NONFREE=1
RUN make -j3 && make install


WORKDIR /git_installed_lib/
# Clone Pangolin along with it's submodules
WORKDIR /git_installed_lib/
RUN git clone --recursive https://github.com/strasdat/Sophus.git
WORKDIR /git_installed_lib/Sophus
RUN git checkout b474f05
WORKDIR /git_installed_lib/Sophus/build/
RUN cmake .. && make -j3 && make install

RUN apt-get update && apt-get install -y libsuitesparse-dev


RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git

WORKDIR /git_installed_lib/Pangolin/
RUN ./scripts/install_prerequisites.sh --dry-run recommended
WORKDIR /git_installed_lib/Pangolin/build/
RUN apt-get update && apt-get install -y libgl1-mesa-dev libglew-dev
RUN cmake .. && make -j3 && make install 


WORKDIR /workspace/

ARG USERNAME=thanhnv
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
