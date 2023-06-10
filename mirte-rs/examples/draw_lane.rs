use cv::{
  detect_lines::detect_line_type,
  line::Colour::{Blue, Green, Red, White, Yellow},
};
use cv::{draw_lines::draw_lines, image::read_image};
use mirte_rs::detect_lane::detect_lane;
use std::collections::HashMap;
use std::env;

#[allow(clippy::expect_used)]
fn main() {
  let mut args = env::args();
  let path = args.nth(1).unwrap_or_else(|| {
    println!("\nError: no input path given!\nExample usage: cargo r -r --example draw_lane ./assets/input_1.jpg\n");
    std::process::exit(1);
  });
  let mut img = read_image(&path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));

  let lines = detect_line_type(&img, &HashMap::new(), vec![Yellow, White])
    .expect("Unable to detect line with cv");
  let lane = detect_lane(&lines);

  draw_lines(
    &mut img,
    &[lines, lane.get_coloured_segments(Green, Blue, Red)].concat(),
  );
}
