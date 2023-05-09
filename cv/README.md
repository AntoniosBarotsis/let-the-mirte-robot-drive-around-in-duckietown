# CV

This crate will contain any Computer Vision utility Mirte might need to figure out how to drive
around Duckietown.

## Compiling

Please follow the instructions [here](https://github.com/twistedfall/opencv-rust#getting-opencv)
to install OpenCV for your platform. It should be noted that WSL seems to be working without
any issues unlike Windows.

For Ubuntu, the TLDR should be to just run:

```sh
# You might also need a "sudo apt-get update"
$ sudo apt install libopencv-dev clang libclang-dev -y
```

## Project Layout

> Everything mentioned here is ran from the root of the **project**, not the root of this *crate*.

The source code is of course in the [`src`](./src/) directory.

There is [one example](./examples/plot_lines.rs) provided which reads in
[an image](../assets/input_real.jpg) taken from Mirte and draws the detected lines in a new file
in the same directory. You can run this with `cargo r --example plot_lines`.

Additionally, there is [a benchmark](./benches/benchmarks/detect_lines.rs) provided which measures
both the time it takes to detect the lines in one image as well as 30 (to simulate 30fps video).
You can run the benchmark with `cargo bench -p cv`, then open
`./target/criterion/report/index.html` in your browser.