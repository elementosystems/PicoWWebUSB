FROM ubuntu:24.04

# Install required build tools and dependencies including the C++ standard library for arm-none-eabi
RUN apt-get update && apt-get upgrade -y --no-install-recommends \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    python3 \
    python3-pip \
    cppcheck \
 && rm -rf /var/lib/apt/lists/*

# Set HOME variable (usually /root in Docker)
ENV HOME=/root

# Clone the Pico SDK into ${HOME}/.pico-sdk so picoVscode file is at ${HOME}/.pico-sdk/cmake/pico-vscode.cmake
RUN git clone --depth=1 https://github.com/raspberrypi/pico-sdk.git ${HOME}/.pico-sdk \
 && cd ${HOME}/.pico-sdk \
 && git submodule update --init

RUN git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git ${HOME}/FreeRTOS-Kernel 

# Set working directory (mount your project here)
WORKDIR /src

# By default, start a shell so you can build your project
CMD [ "bash" ]
