FROM ubuntu:22.04

# Install required build tools and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    libnewlib-arm-none-eabi \
    gcc-arm-none-eabi \
    python3 \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Set HOME variable (usually /root in Docker)
ENV HOME=/root

# Clone the Pico SDK into ${HOME}/.pico-sdk so picoVscode file is at ${HOME}/.pico-sdk/cmake/pico-vscode.cmake
RUN git clone --depth=1 https://github.com/raspberrypi/pico-sdk.git ${HOME}/.pico-sdk \
 && cd ${HOME}/.pico-sdk \
 && git submodule update --init

# Set working directory (mount your project here)
WORKDIR /src

# By default, start a shell so you can build your project
CMD [ "bash" ]