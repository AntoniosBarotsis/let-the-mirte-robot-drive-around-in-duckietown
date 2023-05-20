use cv::draw_lines::{draw_lines, read_image};
use mirte_rs::detect_lane::detect_lane;
use std::env;

fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    println!("\nError: no input path given!\nExample usage: cargo r --example draw_lane ./assets/input_1.jpg\n");
    std::process::exit(1);
  });
  let mut img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));

  let colours = vec![cv::line::Colour::Yellow, cv::line::Colour::White];
  let lines = cv::detect_line_type(&img, colours).expect("Unable to detect line with cv");

  let lane = detect_lane(&lines).expect("Unable to detect the lane");

  let drawn_lines = [lines, lane].concat();
  //lines.push(lane);
  draw_lines(&mut img, &drawn_lines);
}
