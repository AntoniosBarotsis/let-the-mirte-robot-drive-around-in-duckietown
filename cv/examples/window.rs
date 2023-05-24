use cv::draw_lines::{draw_lines, read_image};
use cv::line::Colour::{White, Yellow};
use cv::{detect_line_type, downscale};
use std::time::Instant;

/// Processes the input image from the assets folder and displays it in a window for an easier
/// inspection. The window can also be closed by pressing any button.
fn main() {
  let img = read_image("./assets/input_1.jpg").expect("read image");

  let mut resized = downscale(&img).expect("downscale");

  let now = Instant::now();

  let colours = vec![Yellow, White];
  let lines = detect_line_type(&resized, colours).expect("process image");

  println!("{:?}", now.elapsed());

  draw_lines(&mut resized, &lines);
}
