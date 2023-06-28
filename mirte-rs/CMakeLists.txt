cmake_minimum_required(VERSION 3.0)
project(mirte-rs)

find_package(catkin REQUIRED)
catkin_package()

add_custom_target(mirte-rs
	ALL
	COMMAND cargo build --release --target-dir ${CMAKE_BINARY_DIR}/cargo --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/cargo/release/mirte-rs ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/mirte-rs
	COMMENT "Building mirte-rs"
)

