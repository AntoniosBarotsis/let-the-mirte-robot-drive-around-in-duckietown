use cv::image::{downscale_enhance_hsv, read_image};
use cv::object::detect_obstacles;
use ros::publishers::RosBgPublisher;
use std::env;

use common::debug;

fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    debug!("\nError: no input path given!\nExample usage: cargo r --example detect_obstacles ./assets/obstacles/obstacle_1.png\n");
    std::process::exit(1);
  });
  let img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));
  let usable_img = downscale_enhance_hsv(&img)
    .unwrap_or_else(|_| panic!("Unable to downscale, enhance or convert to hsv"));

  #[allow(clippy::expect_used)]
  let obstacles = detect_obstacles(&usable_img).expect("get obstacles");

  let publisher = RosBgPublisher::get_or_create();
  loop {
    publisher.publish_obstacle(obstacles.clone());
  }
}
