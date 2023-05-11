use cv::process_image;

use opencv::{
  highgui::{imshow, wait_key},
  imgcodecs::{imread, IMREAD_GRAYSCALE},
};

/// Processes the input image from the assets folder and displays it in a window for an easier
/// inspection. The window can also be closed by pressing any button.
fn main() {
  let img = imread("./assets/input_real.jpg", IMREAD_GRAYSCALE).expect("open image");
  let output = process_image(img).expect("process image");

  imshow("test", &output).expect("open window");
  let _res = wait_key(0).expect("keep window open");
}
