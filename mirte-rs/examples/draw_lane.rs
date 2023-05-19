use cv::draw_lines::{draw_lines, read_image};
use mirte_rs::detect_lane::detect_lane;

fn main() {
  let mut img =
    read_image("./assets/input_real.jpg").expect("Unable to get image from /assets/input_real.jpg");

  let colours = vec![cv::line::Colour::Yellow, cv::line::Colour::White];
  let mut lines = cv::detect_line_type(&img, colours).expect("Unable to detect line with cv");

  let lane = detect_lane(&lines).expect("Unable to detect the lane");

  let drawn_lines = [lines, lane].concat();
  //lines.push(lane);
  draw_lines(&mut img, &drawn_lines);
}
