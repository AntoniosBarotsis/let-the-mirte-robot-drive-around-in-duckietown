use cv::cv_error::CvError;
use cv::detect_lines::detect_line_type;
use cv::draw_lines::draw_lines;
use cv::image::{downscale, downscale_enhance_hsv, read_image};
use cv::line::Colour::{White, Yellow};
use std::collections::HashMap;
use std::time::Instant;

/// Processes the input image from the assets folder and displays it in a window for an easier
/// inspection. The window can also be closed by pressing any button.
fn main() -> Result<(), CvError> {
  let img = read_image("./assets/input_1.jpg")?;

  let usable_img = downscale_enhance_hsv(&img)?;

  let now = Instant::now();

  let colours = vec![Yellow, White];
  let lines = detect_line_type(&usable_img, &HashMap::new(), colours)?;

  println!("{:?}", now.elapsed());

  let resized = downscale(&img)?;
  draw_lines(&resized, &lines);

  Ok(())
}
