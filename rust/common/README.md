# Common

This crate generates Mirte's custom ROS messages as well as makes them interchangeable with our
existing Rust structs. It also holds a couple of structs used throughout the project to 
decouple the big `ros` and `cv` crates from one another.

This was done separately from the existing crates to avoid circular dependencies.
