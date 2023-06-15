use opencv::{
  core::{in_range, Point_, Size, Size_, Vec4f, Vector, BORDER_CONSTANT},
  imgproc::{dilate, get_structuring_element, morphology_default_border_value, MORPH_ELLIPSE},
  prelude::{Mat, MatTraitConst, MatTraitConstManual},
  ximgproc::{create_fast_line_detector, FastLineDetector},
};
use std::collections::HashMap;

#[cfg(debug_assertions)]
use crate::image::convert_hsv_to_bgr;

use crate::{
  cv_error::CvError,
  image::crop_image,
  image_part::ImagePart,
  line::{Colour, LineSegment, Point, Threshold},
};

/// Finds lines in the image with a specific colour using the `fast_line_detector` from `openCV`
///
/// * `img` - The HSV image in which the lines need to be detected
/// * `colour` - The colour of which you want to detect the lanes
/// * `line_offset` - The Y offset of where to draw the lines
///
/// Returns gain result of gain vector of lines found on the image with the specified colour
///
/// # Examples
///
/// ```
/// use opencv::{core::Size_, prelude::MatTraitConstManual};
///
/// use cv::{detect_lines::get_lines, image::{convert_to_gray, dbg_mat, downscale}, line::Colour,};
///
/// let mat = dbg_mat("../assets/test_images/test_image_2.png").expect("couldn't get image");
/// let mat_gray = convert_to_gray(&mat).expect("couldn't get gray image");
/// let line_vec = get_lines(&mat_gray, Colour::Yellow, Size_ {width: 320, height: 240,}, 0.0,).expect("couldn't detect a line");
/// assert_eq!(line_vec.len(), 2);
/// assert!(line_vec[0].colour == Colour::Yellow);
/// assert!(line_vec[0].start.x <= 0.60 && line_vec[0].start.x >= 0.40);
/// assert!(line_vec[0].end.x <= 0.60 && line_vec[0].end.x >= 0.40);
/// ```
pub fn get_lines(
  img: &Mat,
  colour: Colour,
  orig_size: Size,
  line_offset: f32,
) -> Result<Vec<LineSegment>, CvError> {
  let mut fast_line_detector = create_fast_line_detector(20, 1.41, 150.0, 350.0, 3, true)
    .map_err(|_e| CvError::LineDetectorCreation)?;

  let mut lines = Vector::<Vec4f>::default();

  fast_line_detector
    .detect(&img, &mut lines)
    .map_err(|_e| CvError::NoLinesDetected)?;

  let mut line_vec: Vec<LineSegment> = Vec::default();

  #[allow(clippy::cast_precision_loss)]
  let width = orig_size.width as f32;
  #[allow(clippy::cast_precision_loss)]
  let height = orig_size.height as f32;

  for line in lines {
    line_vec.push(LineSegment::new(
      colour,
      Point::new(line[0] / width, (line[1] + line_offset) / height),
      Point::new(line[2] / width, (line[3] + line_offset) / height),
    ));
  }
  Ok(line_vec)
}

/// Gets a binary image of only the corresponding colours that are within range. Also works for wrapping around the origin so from 360 degrees to 0 degrees.
///
/// `img` - The image which colour needs to be extracted
/// `threshold` - The colour threshold
///
/// Returns a Result of a binary image where all the pixels that are within the colour range are set to 1
pub fn wrap_in_range(img: &Mat, threshold: Threshold) -> Result<Mat, CvError> {
  let mut colour_img = Mat::default();
  // Check if the hue wraps around the 0 degree border
  if threshold.lower[0] > threshold.upper[0] {
    let max_colour_lower = Mat::from_slice::<u8>(&threshold.lower)?;
    let max_colour_upper = Mat::from_slice::<u8>(&[179, threshold.upper[1], threshold.upper[2]])?;
    let min_colour_lower = Mat::from_slice::<u8>(&[0, threshold.lower[1], threshold.lower[2]])?;
    let min_colour_upper = Mat::from_slice::<u8>(&threshold.upper)?;

    let mut max_img = Mat::default();
    let mut min_img = Mat::default();
    in_range(&img, &max_colour_lower, &max_colour_upper, &mut max_img)?;
    in_range(&img, &min_colour_lower, &min_colour_upper, &mut min_img)?;

    opencv::core::bitwise_or(&max_img, &min_img, &mut colour_img, &Mat::default())?;
    Ok(colour_img)
  } else {
    // Extract the colours
    let colour_low = Mat::from_slice::<u8>(&threshold.lower)?;
    let colour_high = Mat::from_slice::<u8>(&threshold.upper)?;

    in_range(&img, &colour_low, &colour_high, &mut colour_img)?;
    Ok(colour_img)
  }
}

/// Given an HSV image and a vector of colours this method will detect lines in the image for all given colours.
///
/// * `img` - The HSV image in which the lines need to be detected
/// * `thresholds` - A hashmap which maps a `Colour` to a `Threshold`, which determines when a colour
/// is detected.
/// * `colours` - A vector of all the colours of which you want to detect the lines
///
/// Returns a result with a vector of all the lines found in the image
pub fn detect_line_type<S: std::hash::BuildHasher>(
  img: &Mat,
  thresholds: &HashMap<Colour, Threshold, S>,
  colours: Vec<Colour>,
) -> Result<Vec<LineSegment>, CvError> {
  let mut copy_img = Mat::copy(img)?;

  let cropped_img = crop_image(&mut copy_img, ImagePart::Bottom)?;

  let top_img_height = img.rows() - cropped_img.rows();

  #[cfg(debug_assertions)]
  {
    let rgb_img = convert_hsv_to_bgr(&cropped_img)?;
    opencv::highgui::imshow("contrast", &rgb_img)?;
  }

  let mut lines: Vec<LineSegment> = Vec::new();

  for colour in colours {
    let threshold: Threshold = thresholds
      .get(&colour)
      .copied()
      .unwrap_or_else(|| Threshold::by_colour(colour));

    let mut colour_img = wrap_in_range(&cropped_img, threshold)?;

    #[cfg(debug_assertions)]
    {
      match colour {
        Colour::Yellow => {
          opencv::highgui::imshow("yellow", &colour_img)?;
        }
        Colour::White => {
          opencv::highgui::imshow("white", &colour_img)?;
        }
        Colour::Red => {
          opencv::highgui::imshow("red", &colour_img)?;
        }
        _ => (),
      };
    }

    if colour == Colour::Yellow {
      let mut dilated_img = Mat::default();
      let magic = morphology_default_border_value()?;
      let element = get_structuring_element(
        MORPH_ELLIPSE,
        Size_ {
          width: 7,
          height: 7,
        },
        Point_ { x: -1, y: -1 },
      )?;
      dilate(
        &colour_img,
        &mut dilated_img,
        &element,
        Point_ { x: -1, y: -1 },
        2,
        BORDER_CONSTANT,
        magic,
      )?;

      #[cfg(debug_assertions)]
      opencv::highgui::imshow("yellow dilated", &dilated_img)?;

      colour_img = dilated_img;
    }

    // Get the lines of this colour
    // Casting an i32 to an f32 is fine, as the image height is realistically never going to exceed
    // the maximum value of an f32.
    // This takes about 1/6 of the time for 2 colours
    #[allow(clippy::cast_precision_loss)]
    let mut new_lines = get_lines(&colour_img, colour, img.size()?, top_img_height as f32)?;

    lines.append(&mut new_lines);
  }
  Ok(lines)
}
