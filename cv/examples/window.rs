use cv::draw_lines::{draw_lines, read_image};
use cv::line::Colour::{White, Yellow};
use cv::{crop_image, detect_line_type};
use std::time::Instant;

/// Processes the input image from the assets folder and displays it in a window for an easier
/// inspection. The window can also be closed by pressing any button.
fn main() {
  let mut img = read_image("./assets/input_real.jpg").expect("read image");

  let now = Instant::now();

  let colours = vec![Yellow, White];
  let lines = detect_line_type(&img, colours).expect("process image");

  println!("{:?}", now.elapsed());

  let mut draw_img = crop_image(&mut img, cv::image_part::ImagePart::Bottom).expect("crop image");

  draw_lines(&mut draw_img, lines);
}
