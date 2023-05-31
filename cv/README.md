# CV

This crate will contain any Computer Vision utility Mirte might need to figure out how to drive
around Duckietown.

## Compiling

Please follow the instructions [here](https://github.com/twistedfall/opencv-rust#getting-opencv)
to install OpenCV for your platform.

For Ubuntu, the TLDR should be to just run:

```sh
# You might also need a "sudo apt-get update"
$ sudo apt install libopencv-dev clang libclang-dev -y
```

## Project Layout

> Everything mentioned here is ran from the root of the **project**, not the root of this *crate*.

The source code is of course in the [`src`](./src/) directory.

There is one doctest example provided which reads in [an image](../assets/input_real.jpg) taken
from Mirte and draws the detected lines in a new file in the same directory. You can run this with
`cargo test`.

Additionally, there are examples in the [`examples`](./examples/) directory which you can run with

```sh
$ cargo run --example <FILE NAME>
```

Lastly, there is [a benchmark](./benches/benchmarks/detect_lines.rs) provided which measures
both the time it takes to detect the lines in one image as well as 30 (to simulate 30fps video).
You can run the benchmark with `cargo bench -p cv`, then open
`./target/criterion/report/index.html` in your browser.
