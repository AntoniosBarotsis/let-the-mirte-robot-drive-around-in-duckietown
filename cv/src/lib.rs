use cv_error::CvError;
use opencv::prelude::{MatTrait, MatTraitConstManual};
use opencv::{
  core::{Point, Scalar, Vec4f, Vector},
  imgproc::{line, LINE_AA},
  prelude::Mat,
  ximgproc::FastLineDetector,
};

pub mod cv_error;

/// Crops the top half of the image to reduce needed computation.
fn crop_image(img: &mut Mat) -> Result<Mat, CvError> {
  let half_height = img.size().map_err(|e| CvError::Other(e.message))?.height / 2;

  let crop = img
    .adjust_roi(-half_height, 0, 0, 0)
    .map_err(|e| CvError::Other(e.message))?;

  Ok(crop)
}

/// Performs line detection in the passed image. Returns a list of 4d-vectors containing
/// the line coordinates in a `x1, y1, x2, y2` format.
///
/// The image must be in grayscale.
pub fn detect_lines(mut img: Mat) -> Result<Vector<Vec4f>, CvError> {
  let img = crop_image(&mut img)?;

  // Parameter values mostly taken from
  // https://docs.opencv.org/4.x/df/ded/group__ximgproc__fast__line__detector.html
  let mut fast_line_detector =
    opencv::ximgproc::create_fast_line_detector(10, 1.41, 50.0, 50.0, 3, true)
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
/// ## Examples
///
/// You can find an example usage in `../examples/plot_lines.rs`.
pub fn process_image(mut img: Mat) -> Result<Mat, CvError> {
  let lines = detect_lines(img.clone())?;
  let mut cropped_image = crop_image(&mut img)?;

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
