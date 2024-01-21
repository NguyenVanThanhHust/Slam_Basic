FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04
ARG DEBIAN_FRONTEND=noninteractive
ARG CUDA_VERSION=11.8.0

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

RUN apt-get update && apt-get install -y libglfw3-dev libgl-dev libglu-dev

# Install python3.10
WORKDIR /opt/
RUN apt update && add-apt-repository ppa:deadsnakes/ppa -y
RUN apt install -y python3.10-full
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3.10 get-pip.py 

WORKDIR /opt/
RUN python3.10 -m pip install cmake

RUN git clone https://github.com/opencv/opencv.git -b 4.5.2
RUN git clone https://github.com/opencv/opencv_contrib.git -b 4.5.2

WORKDIR /opt//opencv/build/
RUN cmake .. -D BUILD_opencv_java=OFF \
      -D WITH_FFMPEG=ON \
      -D WITH_CUDA=ON \
      -D WITH_CUBLAS=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D CUDA_ARCH_PTX=7.5 \
      -D WITH_NVCUVID=ON \
      -D WITH_CUFFT=ON \
      -D WITH_OPENGL=ON \
      -D WITH_QT=ON \
      -D WITH_IPP=ON \
      -D WITH_TBB=ON \
      -D WITH_EIGEN=ON \
      -D BUILD_opencv_java=OFF \
      -D BUILD_opencv_python=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=OFF \
      -D BUILD_opencv_apps=ON \
      -D BUILD_opencv_aruco=ON \
      -D BUILD_opencv_bgsegm=ON \
      -D BUILD_opencv_bioinspired=ON \
      -D BUILD_opencv_ccalib=ON \
      -D BUILD_opencv_datasets=ON \
      -D BUILD_opencv_dnn_objdetect=ON \
      -D BUILD_opencv_dpm=ON \
      -D BUILD_opencv_fuzzy=OFF \
      -D BUILD_opencv_hfs=OFF \
      -D BUILD_opencv_java_bindings_generator=OFF \
      -D BUILD_opencv_js=OFF \
      -D BUILD_opencv_img_hash=ON \
      -D BUILD_opencv_line_descriptor=ON \
      -D BUILD_opencv_optflow=ON \
      -D BUILD_opencv_phase_unwrapping=ON \
      -D BUILD_opencv_python_bindings_generator=OFF \
      -D BUILD_opencv_reg=ON \
      -D BUILD_opencv_rgbd=ON \
      -D BUILD_opencv_saliency=ON \
      -D BUILD_opencv_shape=ON \
      -D BUILD_opencv_stereo=ON \
      -D BUILD_opencv_stitching=ON \
      -D BUILD_opencv_structured_light=ON \
      -D BUILD_opencv_superres=OFF \
      -D BUILD_opencv_surface_matching=ON \
      -D BUILD_opencv_ts=ON \
      -D BUILD_opencv_cudacodec=ON \
      -D BUILD_opencv_xobjdetect=ON \
      -D BUILD_opencv_xphoto=ON \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D CMAKE_BUILD_TYPE=RELEASE \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D WITH_QT=OFF \
      -D WITH_GTK=ON \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D WITH_CUBLAS=ON \
      -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
      -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
      -D CMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
      -D OpenCL_LIBRARY=/usr/local/cuda-${CUDA_VERSION}/lib64/libOpenCL.so \
      -D OpenCL_INCLUDE_DIR=/usr/local/cuda-${CUDA_VERSION}/include/ \
      -D CMAKE_BUILD_TYPE=RELEASE \
        .. 
RUN make && make install && ldconfig 

WORKDIR /opt/
RUN git clone --recursive https://github.com/strasdat/Sophus.git
WORKDIR /opt/Sophus
RUN git checkout b474f05
WORKDIR /opt/Sophus/build/
RUN cmake .. && make -j3 && make install

RUN apt-get update && apt-get install -y libsuitesparse-dev


WORKDIR /opt/
RUN git clone --recursive https://github.com/RainerKuemmerle/g2o.git 
WORKDIR /opt/g2o/
WORKDIR /opt/g2o/build/
RUN apt-get update && apt-get install -y libgl1-mesa-dev libglew-dev
RUN cmake .. && make -j3 && make install 

WORKDIR /opt/
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
WORKDIR /opt/Pangolin/
RUN apt-get update && apt-get install -y libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev
RUN apt-get update && apt-get install -y libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
RUN apt-get update && apt-get install -y libdc1394-dev libraw1394-dev libopenni-dev
WORKDIR /opt/Pangolin/build/
RUN cmake .. && make -j3 && make install 

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
