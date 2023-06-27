use common::structs::colour::ColourEnum;
use cv::{
  detect_lines::detect_line_type,
  image::{downscale, downscale_enhance_hsv},
};
use cv::{draw_lines::draw_lines, image::read_image};
use mirte_rs::detection::detect_lane;
use std::collections::HashMap;
use std::env;

use common::debug;

#[allow(clippy::expect_used)]
fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    debug!("\nError: no input path given!\nExample usage: cargo r -r --example draw_lane ./assets/input_1.jpg\n");
    std::process::exit(1);
  });
  let img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));

  let usable_img = downscale_enhance_hsv(&img)
    .unwrap_or_else(|_| panic!("Unable to downscale, enhance or convert to hsv"));

  let lines = detect_line_type(
    &usable_img,
    &HashMap::new(),
    vec![ColourEnum::Yellow, ColourEnum::White, ColourEnum::Red],
  )
  .expect("Unable to detect line with cv");
  let lane = detect_lane(&lines);

  let resized =
    downscale(&img).unwrap_or_else(|_| panic!("Unable to downscale image for draing lines"));

  draw_lines(
    &resized,
    &[
      lines,
      lane.get_coloured_segments(ColourEnum::Green, ColourEnum::Orange, ColourEnum::Black),
    ]
    .concat(),
  );
}
