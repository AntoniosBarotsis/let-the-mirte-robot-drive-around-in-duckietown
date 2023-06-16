# antoniosbarotsis/mirte-rs:latest

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

# Get Python
RUN apt-get install python3.7 python3-pip -y

# Source ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Get mirte_msgs
RUN git clone https://github.com/AntoniosBarotsis/mirte-ros-packages
RUN cp ./mirte-ros-packages/mirte_msgs /opt/ros/noetic/share -r
