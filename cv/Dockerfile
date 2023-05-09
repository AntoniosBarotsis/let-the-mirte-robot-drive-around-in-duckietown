FROM rust:1.69.0

RUN apt-get update && apt-get install libopencv-dev clang libclang-dev -y

COPY . .

ENV CARGO_UNSTABLE_SPARSE_REGISTRY=true

RUN cargo b --all-targets
