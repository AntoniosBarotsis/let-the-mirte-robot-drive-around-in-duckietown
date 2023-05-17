use cv::draw_lines::{draw_lines, read_image};

fn main() {
  let mut img = read_image("./assets/input_real.jpg").expect("read image");

  let colours = vec![cv::line::Colour::Yellow, cv::line::Colour::White];
  let lines = cv::detect_line_type(&img, colours).expect("process image");

  draw_lines(&mut img, lines);
}
