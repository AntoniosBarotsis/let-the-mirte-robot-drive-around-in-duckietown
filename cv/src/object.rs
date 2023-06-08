use opencv::{
  core::{in_range, KeyPoint, Point2f, Scalar, Vector},
  features2d::{draw_keypoints, SimpleBlobDetector, SimpleBlobDetector_Params},
  highgui::{imshow, wait_key},
  prelude::{Feature2DTrait, KeyPointTraitConst, Mat},
};

use crate::{
  cv_error::CvError,
  image::{convert_to_hsv, downscale},
};

pub static HSV_DUCK: &[[u8; 3]; 2] = &[[0, 100, 115], [45, 255, 255]];
pub static HSV_MIRTE: &[[u8; 3]; 2] = &[[0, 100, 115], [45, 255, 255]];

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Object {
  Mirte,
  Duck,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obstacle {
  pub location: Point2f,
  pub diameter: f32,
  pub object: Object,
}

impl Obstacle {
  pub fn new(location: Point2f, diameter: f32, object: Object) -> Self {
    Self {
      location,
      diameter,
      object,
    }
  }
}

/// # Panics
pub fn get_obstacles(img: &Mat, object: Object) -> Result<Vec<Obstacle>, CvError> {
  let img_hsv = convert_to_hsv(img)?;

  let downscaled_img = downscale(&img_hsv)?;

  let mut in_range_img = Mat::default();

  let hsv_colour = match object {
    Object::Duck => HSV_DUCK,
    Object::Mirte => HSV_MIRTE,
  };

  let colour_low = Mat::from_slice::<u8>(&hsv_colour[0])?;
  let colour_high = Mat::from_slice::<u8>(&hsv_colour[1])?;

  in_range(
    &downscaled_img,
    &colour_low,
    &colour_high,
    &mut in_range_img,
  )?;

  let mut params: SimpleBlobDetector_Params = SimpleBlobDetector_Params::default()?;

  params.filter_by_color = true;
  params.blob_color = 255;
  params.filter_by_area = true;
  params.min_area = 40.0;
  params.max_area = 100_000.0;
  params.filter_by_inertia = true;
  params.min_inertia_ratio = 0.1;
  params.filter_by_convexity = false;
  params.max_convexity = 0.99;
  params.filter_by_circularity = false;
  params.min_circularity = 0.5;

  let mut blob_detector_ptr = SimpleBlobDetector::create(params)?;

  let mut keypoints = Vector::<KeyPoint>::new();

  blob_detector_ptr.detect(&in_range_img, &mut keypoints, &Mat::default())?;

  let mut output_img = Mat::default();
  draw_keypoints(
    &in_range_img,
    &keypoints,
    &mut output_img,
    Scalar::new(0.0, 0.0, 255.0, 0.0),
    opencv::features2d::DrawMatchesFlags::DEFAULT,
  )?;

  let points: Vec<Obstacle> = keypoints
    .into_iter()
    .map(|x| Obstacle::new(x.pt(), x.size(), object))
    .collect();

  for p in points.clone().into_iter().rev() {
    println!("{p:?}");
  }

  imshow("blob", &output_img)?;
  let _res = wait_key(0)?;

  Ok(points)
}
