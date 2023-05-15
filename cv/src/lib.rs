use cv_error::CvError;
use image_part::ImagePart;
use opencv::{
  core::{Point, Scalar, Vec4f, Vector},
  imgproc::{line, LINE_AA},
  ximgproc::FastLineDetector,
};
use opencv::{
  prelude::{MatTrait, MatTraitConstManual},
  ximgproc::create_fast_line_detector,
};

pub use opencv::prelude::Mat;

pub mod cv_error;
pub mod image_part;

/// Crops the image in half to reduce needed computation.
///
/// The specified image part is the one ***kept*** in the resulting image.
fn crop_image(img: &mut Mat, keep: ImagePart) -> Result<Mat, CvError> {
  let half_height = img.size().map_err(|e| CvError::Other(e.message))?.height / 2;

  let crop = match keep {
    ImagePart::Top => img.adjust_roi(0, -half_height, 0, 0),
    ImagePart::Bottom => img.adjust_roi(-half_height, 0, 0, 0),
  }
  .map_err(|e| CvError::Other(e.message))?;

  Ok(crop)
}

/// Performs line detection in the passed image. Returns a list of 4d-vectors containing
/// the line coordinates in a `x1, y1, x2, y2` format.
///
/// The image must be in grayscale.
pub fn detect_lines(mut img: Mat) -> Result<Vector<Vec4f>, CvError> {
  let img = crop_image(&mut img, ImagePart::Bottom)?;

  // Parameter values mostly taken from
  // https://docs.opencv.org/4.x/df/ded/group__ximgproc__fast__line__detector.html
  let mut fast_line_detector = create_fast_line_detector(20, 1.41, 150.0, 350.0, 3, true)
    .map_err(|_e| CvError::LineDetectorCreation)?;

  let mut lines = Vector::<Vec4f>::default();

  fast_line_detector
    .detect(&img, &mut lines)
    .map_err(|_e| CvError::NoLinesDetected)?;

  Ok(lines)
}

/// Detects lines in the input image, plots them and returns the result.
///
/// The image must be in grayscale.
///
/// This method should be used for testing/debugging only.
///
/// # Examples
///
/// ```
/// use cv::{cv_error::CvError, process_image};
///
/// use opencv::prelude::{MatTrait, MatTraitConstManual};
/// use opencv::{
///   core::Vector,
///   imgcodecs::{self, imread, imwrite, IMREAD_GRAYSCALE},
/// };
///
/// // We need to import the image as grayscale because the FastLineDetector requires it.
/// let img = imread("../assets/input_real.jpg", IMREAD_GRAYSCALE).expect("open image");

/// let output = process_image(img).unwrap();

/// // Save output image.
/// let saved = imwrite("../assets/output.jpg", &output, &Vector::default())
///   .map_err(|e| CvError::IoError(e.message)).unwrap();
///
/// // Make sure that the image was saved correctly
/// assert!(saved);
/// ```
pub fn process_image(mut img: Mat) -> Result<Mat, CvError> {
  let lines = detect_lines(img.clone())?;
  let mut cropped_image = crop_image(&mut img, ImagePart::Bottom)?;

  // Lines contain a list of 4d vectors that, as stated in `FastLineDetector::detect`, holds the
  // values for `x1, y1, x2, y2`.
  for l in lines {
    // Extract the 4 coordinates
    let coords = l.0;

    // Truncation here is fine (and needed) as we are just drawing pixels on the screen.
    #[allow(clippy::cast_possible_truncation)]
    let start_point = Point::new(coords[0] as i32, coords[1] as i32);
    #[allow(clippy::cast_possible_truncation)]
    let end_point = Point::new(coords[2] as i32, coords[3] as i32);

    // OpenCV uses BGR (not RBG) so this is actually red (not that it matters since its greyscale).
    let color = Scalar::new(0.0, 0.0, 255.0, 0.0);

    line(
      &mut cropped_image,
      start_point,
      end_point,
      color,
      5,
      LINE_AA,
      0,
    )
    .map_err(|_e| CvError::Drawing)?;
  }

  Ok(cropped_image)
}

pub fn show_in_window(img: &Mat) {
  let mut img_rgb = Mat::default();

  // This clone here, although seemingly useless, fixes a weird bug that causes artifacts to appear
  // during the color conversion. For more details, refer to:
  // https://github.com/twistedfall/opencv-rust/issues/277
  opencv::imgproc::cvt_color(&img.clone(), &mut img_rgb, opencv::imgproc::COLOR_BGR2RGB, 0)
    .expect("BGR to RGB conversion.");

  opencv::highgui::imshow("img_rgb", &img_rgb).expect("open window");
  let _res = opencv::highgui::wait_key(0).expect("keep window open");
}
