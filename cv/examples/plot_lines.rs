#![allow(clippy::dbg_macro, unused_imports)]

use cv::{cv_error::CvError, process_image};

use opencv::prelude::{MatTrait, MatTraitConstManual};
use opencv::{
  core::Vector,
  imgcodecs::{self, imread, imwrite, IMREAD_GRAYSCALE},
};

fn main() -> Result<(), CvError> {
  // We need to import the image as grayscale because the FastLineDetector requires it.
  let img = imread("./assets/input_real.jpg", IMREAD_GRAYSCALE).expect("open image");

  let output = process_image(img)?;

  // Save output image.
  let _b = imwrite("./assets/output.jpg", &output, &Vector::default())
    .map_err(|e| CvError::IoError(e.message))?;

  Ok(())
}
