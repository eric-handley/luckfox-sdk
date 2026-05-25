FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies from README
RUN apt-get update && apt-get install -y \
    git ssh make gcc gcc-multilib g++-multilib module-assistant expect g++ \
    gawk texinfo libssl-dev bison flex fakeroot cmake unzip gperf autoconf \
    device-tree-compiler libncurses5-dev pkg-config bc python-is-python3 \
    passwd openssl openssh-server openssh-client vim file cpio rsync \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Set up toolchain PATH
ENV PATH="/workspace/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin:${PATH}"

CMD ["/bin/bash"]
