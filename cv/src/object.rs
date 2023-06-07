use opencv::{
  core::{KeyPoint, Scalar, Vector},
  features2d::{draw_keypoints, SimpleBlobDetector, SimpleBlobDetector_Params},
  highgui::{imshow, wait_key},
  prelude::{Feature2DTrait, Mat},
};

use crate::{cv_error::CvError, image::convert_to_gray};

/// # Panics
pub fn get_road_binary(img: &Mat) -> Result<(), CvError> {
  let img_gray = convert_to_gray(img)?;

  let params = SimpleBlobDetector_Params::default()?;

  let mut keypoints: Vector<KeyPoint>;

  let mut blob_detector_ptr = SimpleBlobDetector::create(params)?;

  keypoints = Vector::<KeyPoint>::new();
  println!("before");
  blob_detector_ptr.detect(&img_gray, &mut keypoints, &Mat::default())?;
  println!("after");

  let mut output_img = Mat::default();
  draw_keypoints(
    &img,
    &keypoints,
    &mut output_img,
    Scalar::new(0.0, 0.0, 255.0, 0.0),
    opencv::features2d::DrawMatchesFlags::DEFAULT,
  )?;

  imshow("blob", &output_img)?;
  let _res = wait_key(0)?;

  Ok(())
  // todo!()
}
