use opencv::{
  core::{in_range, KeyPoint, Scalar, Size, Vector},
  features2d::{draw_keypoints, SimpleBlobDetector, SimpleBlobDetector_Params},
  highgui::wait_key,
  prelude::{Feature2DTrait, KeyPointTraitConst, Mat, MatTraitConstManual},
};

use crate::{
  cv_error::CvError,
  image::{convert_to_hsv, downscale},
  line::Point,
};

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_DUCK: &[[u8; 3]; 2] = &[[0, 100, 180], [45, 255, 255]];
pub static HSV_MIRTE: &[[u8; 3]; 2] = &[[70, 70, 100], [100, 255, 255]];

#[derive(Debug, Clone, Copy, PartialEq)]
/// All object types the Mirte bot needs to detect.
pub enum Object {
  Mirte,
  Duck,
}

#[derive(Debug, Clone, Copy, PartialEq)]
/// How obstacles are definded
///
/// * `location` - The location of the object in the image given in percentages
/// * `diameter` - The diamater of the detected object
/// * `object` - The type of object it is. This consist of either `Mirte` or `Duck`
pub struct Obstacle {
  pub location: Point,
  pub diameter: f32,
  pub object: Object,
}

impl Obstacle {
  pub fn new(location: Point, diameter: f32, object: Object) -> Self {
    Self {
      location,
      diameter,
      object,
    }
  }
}

/// Method for detecting all types of Objects. This inlcudes `Duck` and `Mirte`.
///
/// * `input_img` - The image in which obstacles need to be detected
///
/// Returns a Result with a vector of Obstacles containing its location, diameter and what Type of object it is.
///
/// # Example
///
/// ```
/// use cv::image::dbg_mat;
/// use cv::object::{get_obstacles, Object, Obstacle};
///
/// let mat = dbg_mat("../assets/obstacles/obstacle_15.jpeg").expect("couldn't get the image");
/// let obstacles = get_obstacles(&mat).expect("was not able to get any obstacles");
/// assert_eq!(obstacles[0].object, Object::Mirte);
/// assert_eq!(obstacles[1].object, Object::Duck);
/// assert!(obstacles[0].location.x < 0.55 && obstacles[0].location.x > 0.45);
/// assert!(obstacles[0].location.y < 0.5 && obstacles[0].location.y > 0.4);
/// assert!(obstacles[1].location.x < 0.55 && obstacles[1].location.x > 0.45);
/// assert!(obstacles[1].location.y < 0.35 && obstacles[1].location.y > 0.25);
/// ```
pub fn get_obstacles(input_img: &Mat) -> Result<Vec<Obstacle>, CvError> {
  let img_hsv = convert_to_hsv(input_img)?;

  let img = downscale(&img_hsv)?;
  let img_size = img.size()?;

  // imshow("hsv", &img)?;

  let mirtes = get_mirtes(&img, img_size)?;
  let duckies = get_duckies(&img, img_size)?;

  let _res = wait_key(0)?;

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
/// use cv::image::{convert_to_hsv, dbg_mat, downscale};
/// use cv::object::{get_duckies, Object, Obstacle};
///
/// use opencv::{
///   core::Size,
///   prelude::{Mat, MatTraitConstManual},
/// };
///
/// let mat = dbg_mat("../assets/obstacles/obstacle_15.jpeg").expect("couldn't get image");
/// let img_hsv = convert_to_hsv(&mat).expect("couldn't conver to HSV");
/// let img = downscale(&img_hsv).expect("couldn't downscale image");
/// let img_size = img.size().expect("couldn't get size of image");
///
/// let obstacles = get_duckies(&img, img_size).expect("couldn't get dukies :(");
/// assert_eq!(obstacles[0].object, Object::Duck);
/// assert!(obstacles[0].location.x < 0.55 && obstacles[0].location.x > 0.45);
/// assert!(obstacles[0].location.y < 0.35 && obstacles[0].location.y > 0.25);
/// ```
pub fn get_duckies(img: &Mat, img_size: Size) -> Result<Vec<Obstacle>, CvError> {
  // Extract the colours
  let colour_low = Mat::from_slice::<u8>(&HSV_DUCK[0])?;
  let colour_high = Mat::from_slice::<u8>(&HSV_DUCK[1])?;

  let mut colour_img = Mat::default();
  in_range(&img, &colour_low, &colour_high, &mut colour_img)?;

  let mut params: SimpleBlobDetector_Params = SimpleBlobDetector_Params::default()?;
  params.filter_by_color = true;
  params.blob_color = 255;
  params.filter_by_area = true;
  params.min_area = 40.0;
  params.max_area = 100_000.0;
  params.filter_by_inertia = true;
  params.min_inertia_ratio = 0.1;
  params.filter_by_convexity = false;
  params.filter_by_circularity = false;

  // imshow("inrange duck", &colour_img)?;

  let points = detect_obstacles_with_params(&colour_img, params, img_size, Object::Duck)?;
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
/// use cv::image::{convert_to_hsv, dbg_mat, downscale};
/// use cv::object::{get_mirtes, Object, Obstacle};
///
/// use opencv::{
///   core::Size,
///   prelude::{Mat, MatTraitConstManual},
/// };
///
/// let mat = dbg_mat("../assets/obstacles/obstacle_15.jpeg").expect("couldn't get image");
/// let img_hsv = convert_to_hsv(&mat).expect("couldn't conver to HSV");
/// let img = downscale(&img_hsv).expect("couldn't downscale image");
/// let img_size = img.size().expect("couldn't get size of image");
/// let obstacles = get_mirtes(&img, img_size).expect("couldn't find mirte bots");
///
/// assert_eq!(obstacles[0].object, Object::Mirte);
/// assert!(obstacles[0].location.x < 0.55 && obstacles[0].location.x > 0.45);
/// assert!(obstacles[0].location.y < 0.5 && obstacles[0].location.y > 0.4);
/// ```
pub fn get_mirtes(img: &Mat, img_size: Size) -> Result<Vec<Obstacle>, CvError> {
  // Extract the colours
  let colour_low = Mat::from_slice::<u8>(&HSV_MIRTE[0])?;
  let colour_high = Mat::from_slice::<u8>(&HSV_MIRTE[1])?;

  let mut colour_img = Mat::default();
  in_range(&img, &colour_low, &colour_high, &mut colour_img)?;

  // let colour_img = denoise(&or_img)?;

  let mut params: SimpleBlobDetector_Params = SimpleBlobDetector_Params::default()?;

  params.filter_by_color = true;
  params.blob_color = 255;
  params.filter_by_area = true;
  params.min_area = 20.0;
  params.max_area = 100_000.0;
  params.filter_by_inertia = true;
  params.min_inertia_ratio = 0.1;
  params.filter_by_convexity = false;
  params.max_convexity = 0.99;
  params.filter_by_circularity = false;
  params.min_circularity = 0.5;

  // imshow("inrange mirte", &colour_img)?;

  let points = detect_obstacles_with_params(&colour_img, params, img_size, Object::Mirte)?;
  Ok(points)
}

// fn denoise(input_img: &Mat) -> Result<Mat, CvError> {
//   let mut eroded_img = Mat::default();
//   let magic = morphology_default_border_value()?;
//   let element = get_structuring_element(
//     MORPH_ELLIPSE,
//     Size_ {
//       width: 4,
//       height: 4,
//     },
//     Point_ { x: -1, y: -1 },
//   )?;
//   erode(
//     &input_img,
//     &mut eroded_img,
//     &element,
//     Point_ { x: -1, y: -1 },
//     1,
//     BORDER_CONSTANT,
//     magic,
//   )?;

//   imshow("eroded mirte", &eroded_img)?;

//   let mut dilated_img = Mat::default();
//   let magic = morphology_default_border_value()?;
//   let element = get_structuring_element(
//     MORPH_ELLIPSE,
//     Size_ {
//       width: 20,
//       height: 20,
//     },
//     Point_ { x: -1, y: -1 },
//   )?;
//   dilate(
//     &eroded_img,
//     &mut dilated_img,
//     &element,
//     Point_ { x: -1, y: -1 },
//     1,
//     BORDER_CONSTANT,
//     magic,
//   )?;

//   Ok(dilated_img)
// }

/// Given an image and the `simpleBlobDetector` parameters it will construct a `simpleBlobDetector` and use that to detect a given object on the screen.
///
/// * `img` - The image in which the object needs to be detected
/// * `params` - The `simpleBlobDetector` parameters
/// * `img_size` - The size of the image
/// * `object` - The type of object it is. This includes `Duck` and `Mirte`
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
  let width = img_size.width as f32;
  #[allow(clippy::cast_precision_loss)]
  let height = img_size.height as f32;

  let mut output_img = Mat::default();
  draw_keypoints(
    &img,
    &keypoints,
    &mut output_img,
    Scalar::new(0.0, 0.0, 255.0, 0.0),
    opencv::features2d::DrawMatchesFlags::DEFAULT,
  )?;

  // imshow("blob", &output_img)?;

  let obstacles: Vec<Obstacle> = keypoints
    .into_iter()
    .map(|x| {
      Obstacle::new(
        Point::new(x.pt().x / width, x.pt().y / height),
        x.size(),
        object,
      )
    })
    .collect();

  for ob in obstacles.clone().into_iter().rev() {
    println!("{ob:?}");
  }

  Ok(obstacles)
}
