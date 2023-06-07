use std::mem;

use opencv::{
  core::{KeyPoint, Scalar, Vector},
  features2d::{draw_keypoints, Feature2D, SimpleBlobDetector, SimpleBlobDetector_Params},
  highgui::{imshow, wait_key},
  prelude::{Boxed, Feature2DTrait, Mat},
};

use crate::{cv_error::CvError, image::convert_to_gray};

/// # Panics
pub fn get_road_binary(img: &Mat) -> Result<(), CvError> {
  let img_gray = convert_to_gray(img)?;

  let mut blob_detector: Feature2D;
  let params = SimpleBlobDetector_Params::default()?;

  let mut keypoints: Vector<KeyPoint>;

  #[allow(unsafe_code)]
  unsafe {
    let mut blob_detector_ptr = SimpleBlobDetector::create(params)?;
    let raw_ptr = blob_detector_ptr.as_raw_mut_Feature2D();
    blob_detector = Feature2D::from_raw(raw_ptr);

    keypoints = Vector::<KeyPoint>::new();
    println!("before");
    blob_detector.detect(&img_gray, &mut keypoints, &Mat::default())?;
    println!("after");

    #[allow(clippy::mem_forget)]
    mem::forget(blob_detector);
  };

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
