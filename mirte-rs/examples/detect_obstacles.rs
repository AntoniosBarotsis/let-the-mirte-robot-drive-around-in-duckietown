use cv::image::read_image;
use cv::object::get_obstacles;
use ros::publishers::RosBgPublisher;
use std::env;

fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    println!("\nError: no input path given!\nExample usage: cargo r --example detect_obstacles ./assets/obstacles/obstacle_1.png\n");
    std::process::exit(1);
  });
  let img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));

  #[allow(clippy::expect_used)]
  let obstacles = get_obstacles(&img).expect("get obstacles");

  let publisher = RosBgPublisher::get_or_create();
  loop {
    publisher.publish_obstacle(obstacles.clone());
  }
}
