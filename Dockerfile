FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

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
 && rm -rf /var/lib/apt/lists/*

RUN git clone https://gitlab.kitware.com/cmake/cmake.git && \
cd cmake && git checkout tags/v3.20.2 && ./bootstrap --parallel=8 && make -j8 && make install && \
cd .. && rm -rf cmake


# Create a working directory
RUN cd ~/
RUN mkdir -p ~/projects/
WORKDIR /projects

RUN git clone https://github.com/opencv/opencv.git -b 4.5.2
RUN git clone https://github.com/opencv/opencv_contrib.git -b 4.5.2

RUN mkdir -p ~/projects/opencv/build/
WORKDIR /projects/opencv/build/
RUN cmake .. -D BUILD_opencv_java=OFF -D BUILD_opencv_python=0  -D BUILD_opencv_python2=0 -D BUILD_opencv_python3=0 -DOPENCV_EXTRA_MODULES_PATH= ../../opencv_contrib/modules/ -D OPENCV_ENABLE_NONFREE=1
RUN make -j3


# Create a non-root user and switch to it
RUN adduser --disabled-password --gecos '' --shell /bin/bash user \
 && chown -R user:user /projects
RUN echo "user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-user
USER user


# All users can use /home/user as their home directory
ENV HOME=/home/user
RUN chmod 777 /home/user

# Install Miniconda and Python 3.7
ENV CONDA_AUTO_UPDATE_CONDA=false
ENV PATH=/home/user/miniconda/bin:$PATH
RUN curl -sLo ~/miniconda.sh https://repo.continuum.io/miniconda/Miniconda3-py37_4.9.2-Linux-x86_64.sh \
 && chmod +x ~/miniconda.sh \
 && ~/miniconda.sh -b -p ~/miniconda \
 && rm ~/miniconda.sh \
 && conda install -y python==3.7.10 \
 && conda clean -ya

# CUDA 10.2-specific steps
RUN conda install -y -c pytorch \
    cudatoolkit=10.2 \
    "pytorch=1.5.0=py3.7_cuda10.2.89_cudnn7.6.5_0" \
    "torchvision=0.6.0=py37_cu102" \
 && conda clean -ya



# Launch terminator
CMD ["terminator"]