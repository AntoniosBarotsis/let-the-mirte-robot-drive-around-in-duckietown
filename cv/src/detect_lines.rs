use opencv::{
  core::{in_range, Point_, Size, Size_, Vec4f, Vector, BORDER_CONSTANT},
  imgproc::{
    cvt_color, dilate, get_structuring_element, morphology_default_border_value, COLOR_RGB2HSV,
    MORPH_ELLIPSE,
  },
  prelude::{Mat, MatTraitConst, MatTraitConstManual},
  ximgproc::{create_fast_line_detector, FastLineDetector},
};

#[cfg(debug_assertions)]
use crate::image::convert_to_rgb;

use crate::{
  cv_error::CvError,
  image::{crop_image, enhance_contrast},
  image_part::ImagePart,
  line::{get_colour, Colour, LineSegment, Point},
};

/// Finds lines in the image with a specific colour using the `fast_line_detector` from `openCV`
///
/// * `img` - The images of which the lines need to be detected
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

/// Given a image and a vector of colours this method will detect lines in the image for all given colours.
///
/// * `img` - The image in which the lines need to be detected
/// * `colours` - A vector of all the colours if which you want to detect the lines
///
/// Returns a result with a vector of all the lines found in the image
pub fn detect_line_type(img: &Mat, colours: Vec<Colour>) -> Result<Vec<LineSegment>, CvError> {
  let mut copy_img = Mat::copy(img)?;

  let cropped_img = crop_image(&mut copy_img, ImagePart::Bottom)?;

  let top_img_height = img.rows() - cropped_img.rows();

  // Contrast stretching
  let contrast_img = enhance_contrast(&cropped_img)?;

  #[cfg(debug_assertions)]
  {
    let rgb_img = convert_to_rgb(&contrast_img)?;
    opencv::highgui::imshow("contrast", &rgb_img)?;
  }

  let mut hsv_img = Mat::default();

  // Converting colour takes about half of the time of this funciton
  // Colour code should be `COLOR_BGR2HSV` when image file is used.
  // Colour code should be `COLOR_RGB2HSV` when ROS image is used.
  cvt_color(&contrast_img, &mut hsv_img, COLOR_RGB2HSV, 0)?;

  let mut lines: Vec<LineSegment> = Vec::new();

  for colour_enum in colours {
    let colour: &[[u8; 3]; 2] = get_colour(colour_enum);

    let mut colour_img = Mat::default();
    // Check if the hue wraps around the 0 degree border
    if colour[0][0] > colour[1][0] {
      let range_lower = &[colour[0], [179, colour[1][1], colour[1][2]]];
      let range_upper = &[[0, colour[0][1], colour[0][2]], colour[1]];

      let colour_lower1 = Mat::from_slice::<u8>(&range_lower[0])?;
      let colour_lower2 = Mat::from_slice::<u8>(&range_lower[1])?;
      let colour_upper1 = Mat::from_slice::<u8>(&range_upper[0])?;
      let colour_upper2 = Mat::from_slice::<u8>(&range_upper[1])?;

      let mut img_lower = Mat::default();
      let mut img_upper = Mat::default();
      in_range(&hsv_img, &colour_lower1, &colour_lower2, &mut img_lower)?;
      in_range(&hsv_img, &colour_upper1, &colour_upper2, &mut img_upper)?;

      opencv::core::bitwise_or(&img_lower, &img_upper, &mut colour_img, &Mat::default())?;
    } else {
      // Extract the colours
      let colour_low = Mat::from_slice::<u8>(&colour[0])?;
      let colour_high = Mat::from_slice::<u8>(&colour[1])?;

      in_range(&hsv_img, &colour_low, &colour_high, &mut colour_img)?;
    }

    #[cfg(debug_assertions)]
    {
      match colour_enum {
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

    if colour_enum == Colour::Yellow {
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
    let mut new_lines = get_lines(&colour_img, colour_enum, img.size()?, top_img_height as f32)?;

    lines.append(&mut new_lines);
  }
  Ok(lines)
}
