use cv::process_image;

use opencv::{
  core::{Point, Size_},
  highgui::{imshow, wait_key},
  imgcodecs::{imread, IMREAD_GRAYSCALE},
  imgproc::blur,
  prelude::Mat,
};

/// Demonstrates how to apply blur to an image.
fn main() {
  let img = imread("./assets/input_real.jpg", IMREAD_GRAYSCALE).expect("open image");

  let mut blurred = Mat::default();

  blur(
    &img,
    &mut blurred,
    Size_ {
      width: 18,
      height: 10,
    },
    Point::new(-1, -1),
    opencv::core::BORDER_DEFAULT,
  )
  .expect("Blur");

  let output = process_image(blurred).expect("process image");
  imshow("test", &output).expect("open window");
  let _res = wait_key(0).expect("keep window open");
}
