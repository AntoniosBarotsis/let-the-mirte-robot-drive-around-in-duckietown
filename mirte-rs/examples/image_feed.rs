use cv::line::Colour::{White, Yellow};
use cv::{detect_lines::detect_line_type, image::read_image};
use ros::publishers::RosBgPublisher;
use std::{env, time::Instant};

#[allow(clippy::expect_used)]
fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    println!("\nError: no input path given!\nExample usage: cargo r -r --example draw_lane ./assets/input_1.jpg\n");
    std::process::exit(1);
  });
  let img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));

  let lines = detect_line_type(&img, vec![Yellow, White]).expect("Unable to detect line with cv");

  let worker = RosBgPublisher::new();

  loop {
    let start = Instant::now();

    worker
      .publish_line_segment(lines.clone())
      .expect("Publish works");

    println!("{:?}", start.elapsed());
  }
}
