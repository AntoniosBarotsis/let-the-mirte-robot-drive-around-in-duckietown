use common::{
  geometry_msgs::Point,
  mirte_duckietown_msgs::{Object, Obstacle},
  structs::threshold::Threshold,
};
use opencv::{
  core::{KeyPoint, Size, Vector},
  features2d::{SimpleBlobDetector, SimpleBlobDetector_Params},
  prelude::{Feature2DTrait, KeyPointTraitConst, Mat, MatTraitConstManual},
};

use crate::{cv_error::CvError, detect_lines::wrap_in_range};

#[cfg(debug_assertions)]
use opencv::{
  core::Scalar,
  features2d::draw_keypoints,
  highgui::{imshow, wait_key},
};

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_DUCK: Threshold = Threshold {
  lower: [170, 110, 110],
  upper: [45, 255, 255],
};
pub static HSV_MIRTE: Threshold = Threshold {
  lower: [70, 100, 30],
  upper: [100, 255, 255],
};

/// Method for detecting all types of Objects. This inlcudes `Duck` and `Mirte`.
///
/// * `input_img` - The image in which obstacles need to be detected
///
/// Returns a Result with a vector of Obstacles containing its location, diameter and what Type of object it is.
///
/// # Example
///
/// ```
/// use cv::image::{dbg_mat, downscale_enhance_hsv};
/// use cv::object::detect_obstacles;
/// use common::mirte_duckietown_msgs::{Object, Obstacle};
/// use common::geometry_msgs::Point;
///
/// let mat = dbg_mat("../assets/obstacles/obstacle_15.jpeg").expect("couldn't get the image");
/// let pros_img = downscale_enhance_hsv(&mat).expect("couldn't apply pipeline");
/// let obstacles = detect_obstacles(&pros_img).expect("was not able to get any obstacles");
/// let duckstacles: Vec<Obstacle> = obstacles.clone().into_iter().filter(|x| {x.object == Object { type_: Object::DUCK }}).collect();
/// let mirtstacles: Vec<Obstacle> = obstacles.clone().into_iter().filter(|x| {x.object == Object { type_: Object::MIRTE }}).collect();
///
/// assert_eq!(duckstacles.len(), 1);
/// assert!(mirtstacles.len() >= 1);
/// assert_eq!(duckstacles[0].object, Object { type_: Object::DUCK });
/// assert_eq!(mirtstacles[0].object, Object { type_: Object::MIRTE });
/// assert!(mirtstacles[0].location.x > 0.40 && mirtstacles[0].location.x < 0.50);
/// assert!(mirtstacles[0].location.y > 0.50 && mirtstacles[0].location.y < 0.65);
/// assert!(duckstacles[0].location.x > 0.35 && duckstacles[0].location.x < 0.45);
/// assert!(duckstacles[0].location.y > 0.25 && duckstacles[0].location.y < 0.40);
/// ```
pub fn detect_obstacles(img: &Mat) -> Result<Vec<Obstacle>, CvError> {
  let img_size = img.size()?;

  let mirtes = get_mirtes(img, img_size)?;
  let duckies = get_duckies(img, img_size)?;

  Ok([mirtes, duckies].concat())
}

/// Method for detecting the `Duckie` Obstacle.
///
/// * `img` - The image in which objects needs to be detected
/// * `img_size` - The size of the image
///
/// Returns an Result with a vector of the detected Duckies containing its location, Diameter and that it is a Duckie type.
///
/// # Example
///
/// ```
/// use cv::image::{dbg_mat, downscale_enhance_hsv};
/// use cv::object::get_duckies;
/// use common::mirte_duckietown_msgs::{Object, Obstacle};
/// use common::geometry_msgs::Point;
///
/// use opencv::{
///   core::Size,
///   prelude::{Mat, MatTraitConstManual},
/// };
///
/// let mat = dbg_mat("../assets/obstacles/obstacle_15.jpeg").expect("couldn't get image");
/// let pros_img = downscale_enhance_hsv(&mat).expect("couldn't apply pipeline");
/// let img_size = pros_img.size().expect("couldn't get size of image");
/// let obstacles = get_duckies(&pros_img, img_size).expect("couldn't get dukies :(");
///
/// assert_eq!(obstacles.len(), 1);
/// assert_eq!(obstacles[0].object, Object { type_: Object::DUCK });
/// assert!(obstacles[0].location.x > 0.30 && obstacles[0].location.x < 0.40);
/// assert!(obstacles[0].location.y > 0.30 && obstacles[0].location.y < 0.40);
/// ```
pub fn get_duckies(img: &Mat, img_size: Size) -> Result<Vec<Obstacle>, CvError> {
  // Extract the colours

  let colour_img = wrap_in_range(img, HSV_DUCK)?;

  let mut params: SimpleBlobDetector_Params = SimpleBlobDetector_Params::default()?;
  // all parameters for detecting duckies
  params.filter_by_color = true;
  params.blob_color = 255;
  params.filter_by_area = true;
  params.min_area = 1000.0; // 100 right now. Might change later if problem occur
  params.max_area = 12_800.0; // 1/6th of the image
  params.filter_by_inertia = true;
  params.min_inertia_ratio = 0.25;
  params.filter_by_convexity = false;
  params.filter_by_circularity = true;
  params.min_circularity = 0.2;

  let points = detect_obstacles_with_params(
    &colour_img,
    params,
    img_size,
    Object {
      type_: Object::DUCK,
    },
  )?;
  Ok(points)
}

/// Method for detecting Mirte bots
///
/// * `img` - The image in which objects needs to be detected
/// * `img_size` - The size of the image
///
/// Returns an Result with a vector of the detected Mirte bots containing its location, Diameter and that it is a Mirte bot
///
/// # Example
///
/// ```
/// use cv::image::{dbg_mat, downscale_enhance_hsv};
/// use cv::object::get_mirtes;
/// use common::mirte_duckietown_msgs::{Object, Obstacle};
/// use common::geometry_msgs::Point;
///
/// use opencv::{
///   core::Size,
///   prelude::{Mat, MatTraitConstManual},
/// };
///
/// let mat = dbg_mat("../assets/obstacles/obstacle_15.jpeg").expect("couldn't get image");
/// let pros_img = downscale_enhance_hsv(&mat).expect("couldn't apply pipeline");
/// let img_size = pros_img.size().expect("couldn't get size of image");
/// let obstacles = get_mirtes(&pros_img, img_size).expect("couldn't find mirte bots");
///
/// assert!(obstacles.len() >= 1);
/// assert_eq!(obstacles[0].object, Object { type_: Object::MIRTE });
/// assert!(obstacles[0].location.x > 0.40 && obstacles[0].location.x < 0.50);
/// assert!(obstacles[0].location.y > 0.50 && obstacles[0].location.y < 0.65);
/// ```
pub fn get_mirtes(img: &Mat, img_size: Size) -> Result<Vec<Obstacle>, CvError> {
  // Extract the colours
  let colour_img = wrap_in_range(img, HSV_MIRTE)?;

  let mut params: SimpleBlobDetector_Params = SimpleBlobDetector_Params::default()?;
  // all paramters for detecting mirte bots
  params.filter_by_color = true;
  params.blob_color = 255;
  params.filter_by_area = true;
  params.min_area = 20.0;
  params.max_area = 3072.0; // 1/25 of the image
  params.filter_by_inertia = true;
  params.min_inertia_ratio = 0.1;
  params.filter_by_convexity = true;
  params.filter_by_circularity = false;
  params.min_circularity = 0.5;

  let points = detect_obstacles_with_params(
    &colour_img,
    params,
    img_size,
    Object {
      type_: Object::MIRTE,
    },
  )?;
  Ok(points)
}

/// Given an image and the `simpleBlobDetector` parameters it will construct a `simpleBlobDetector` and use that to detect a given object on the screen.
///
/// * `img` - The image in which the object needs to be detected
/// * `params` - The `simpleBlobDetector` parameters
/// * `img_size` - The size of the image
/// * `object` - The type of object it is.
///
/// Returns a result with a vector of the detected object type containing its location, diameter and the type of object it is.
fn detect_obstacles_with_params(
  img: &Mat,
  params: SimpleBlobDetector_Params,
  img_size: Size,
  object: Object,
) -> Result<Vec<Obstacle>, CvError> {
  let mut blob_detector_ptr = SimpleBlobDetector::create(params)?;

  let mut keypoints = Vector::<KeyPoint>::new();

  blob_detector_ptr.detect(&img, &mut keypoints, &Mat::default())?;

  #[allow(clippy::cast_precision_loss)]
  let width = f64::from(img_size.width);
  #[allow(clippy::cast_precision_loss)]
  let height = f64::from(img_size.height);

  #[cfg(debug_assertions)]
  {
    let mut output_img = Mat::default();
    draw_keypoints(
      &img,
      &keypoints,
      &mut output_img,
      Scalar::new(0.0, 0.0, 255.0, 0.0),
      opencv::features2d::DrawMatchesFlags::DEFAULT,
    )?;

    imshow("obstacle", &output_img)?;
    #[allow(clippy::expect_used)]
    let _res = wait_key(0).expect("keep window open");
  }

  let obstacles: Vec<Obstacle> = keypoints
    .into_iter()
    .map(|x| {
      Obstacle::new(
        Point::new(f64::from(x.pt().x) / width, f64::from(x.pt().y) / height),
        x.size(),
        object,
      )
    })
    .collect();

  #[cfg(debug_assertions)]
  for ob in obstacles.clone().into_iter().rev() {
    common::debug!("{ob:?}");
  }

  Ok(obstacles)
}
