use common::debug;
use common::structs::colour::ColourEnum;
use cv::detect_lines::detect_line_type;
use cv::draw_lines::draw_lines;
use cv::image::{downscale, read_image};
use cv::{cv_error::CvError, image::downscale_enhance_hsv};
use std::collections::HashMap;
#[cfg(debug_assertions)]
use std::time::Instant;

/// Processes the input image from the assets folder and displays it in a window for an easier
/// inspection. The window can also be closed by pressing any button.
fn main() -> Result<(), CvError> {
  let img = read_image("./assets/input_1.jpg")?;

  let usable_img = downscale_enhance_hsv(&img)?;

  #[cfg(debug_assertions)]
  let now = Instant::now();

  let colours = vec![ColourEnum::Yellow, ColourEnum::White];
  let lines = detect_line_type(&usable_img, &HashMap::new(), colours)?;

  debug!("{:?}", now.elapsed());

  let resized = downscale(&img)?;
  draw_lines(&resized, &lines);

  Ok(())
}
