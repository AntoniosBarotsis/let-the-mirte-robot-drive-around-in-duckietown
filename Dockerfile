# antoniosbarotsis/mirte-rs:opencv-ros-py-v3

FROM ros:noetic-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive

# Update default packages
RUN apt-get update

# Get Ubuntu packages
RUN apt-get install -y \
    build-essential \
    curl \
    libopencv-dev \
    clang \
    libclang-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Update new packages
RUN apt-get update

# Get Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

ENV PATH="/root/.cargo/bin:${PATH}"

ENV CARGO_REGISTRIES_CRATES_IO_PROTOCOL=sparse

# Get Python and Maturin
RUN apt-get install python3.7 python3-pip -y
RUN pip3 install maturin patchelf
# Source ROS setup
RUN echo "source catkin_ws/devel/setup.sh" >> ~/.bashrc

# Get mirte_msgs
RUN git clone https://github.com/AntoniosBarotsis/mirte-ros-packages
RUN cp ./mirte-ros-packages/mirte_msgs ~/catkin_ws/src/ -r
