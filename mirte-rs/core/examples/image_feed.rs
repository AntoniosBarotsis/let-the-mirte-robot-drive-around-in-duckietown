use common::debug;
use cv::image::read_image;
use std::collections::HashMap;
use std::env;

#[allow(clippy::expect_used)]
fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    debug!("\nError: no input path given!\nExample usage: cargo r -r --example image_feed ./assets/input_1.jpg\n");
    std::process::exit(1);
  });
  let img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));

  loop {
    core::process_mat(&img.clone(), &HashMap::new());
  }
}
