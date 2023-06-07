use cv::image::read_image;
use cv::object::get_road_binary;
use std::env;

fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    println!("\nError: no input path given!\nExample usage: cargo r --example detect_obstacles ./assets/input_1.jpg\n");
    std::process::exit(1);
  });
  let img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));

  #[allow(clippy::expect_used)]
  get_road_binary(&img).expect("get obstacles");
}
