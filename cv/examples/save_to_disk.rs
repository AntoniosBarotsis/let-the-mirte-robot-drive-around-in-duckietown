use cv::{cv_error::CvError, process_image};

use opencv::{
  core::Vector,
  imgcodecs::{imread, imwrite, IMREAD_GRAYSCALE},
};

/// This example performs line detection and saves the image to disk as "`assets/input_real.jpg`".
fn main() {
  // We need to import the image as grayscale because the FastLineDetector requires it.
  let img = imread("./assets/input_real.jpg", IMREAD_GRAYSCALE).expect("open image");

  let output = process_image(img).expect("Process image");

  // Save output image.
  let saved = imwrite("./assets/output.jpg", &output, &Vector::default())
    .map_err(|e| CvError::Io(e.message))
    .expect("Save works");

  // Make sure that the image was saved correctly
  assert!(saved);
}
