use cv::line::Colour::{White, Yellow};
use cv::{detect_lines::detect_line_type, image::read_image};
use mirte_rs::detect_lane::detect_lane;
use std::env;

/// Detects line types and lanes + publishes them to their ROS topics.
#[allow(clippy::expect_used)]
fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    println!("\nError: no input path given!\nExample usage: cargo r -r --example image_feed ./assets/input_1.jpg\n");
    std::process::exit(1);
  });
  let img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));

  loop {
    let lines = detect_line_type(&img, vec![Yellow, White]).expect("Unable to detect line with cv");
    let _lane = detect_lane(&lines);
  }
}
