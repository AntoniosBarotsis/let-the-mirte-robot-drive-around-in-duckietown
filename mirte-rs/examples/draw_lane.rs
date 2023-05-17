use cv::draw_lines::{draw_lines, read_image};
use mirte_rs::detect_lane::detect_lane;

fn main() {
  let mut img = read_image("./assets/input_real.jpg").expect("read image");

  let colours = vec![cv::line::Colour::Yellow, cv::line::Colour::White];
  let mut lines = cv::detect_line_type(&img, colours).expect("process image");

  let lane = detect_lane(&lines).expect("detect lane");

  lines.push(lane);
  draw_lines(&mut img, &lines);
}
