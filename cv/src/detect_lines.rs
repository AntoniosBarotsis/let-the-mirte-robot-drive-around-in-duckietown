use opencv::{
  core::{in_range, Size, Vec4f, Vector},
  imgproc::{cvt_color, COLOR_RGB2HSV},
  prelude::{Mat, MatTraitConst, MatTraitConstManual},
  ximgproc::{create_fast_line_detector, FastLineDetector},
};

#[cfg(debug_assertions)]
use crate::image::convert_to_rgb;

use crate::{
  cv_error::CvError,
  image::{crop_image, enhance_contrast},
  image_part::ImagePart,
  line::{get_colour, Colour, Line, Pos},
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
) -> Result<Vec<Line>, CvError> {
  let mut fast_line_detector = create_fast_line_detector(20, 1.41, 150.0, 350.0, 3, true)
    .map_err(|_e| CvError::LineDetectorCreation)?;

  let mut lines = Vector::<Vec4f>::default();

  fast_line_detector
    .detect(&img, &mut lines)
    .map_err(|_e| CvError::NoLinesDetected)?;

  let mut line_vec: Vec<Line> = Vec::default();

  #[allow(clippy::cast_precision_loss)]
  let width = orig_size.width as f32;
  #[allow(clippy::cast_precision_loss)]
  let height = orig_size.height as f32;

  for line in lines {
    line_vec.push(Line::new(
      colour,
      Pos::new(line[0] / width, (line[1] + line_offset) / height),
      Pos::new(line[2] / width, (line[3] + line_offset) / height),
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
pub fn detect_line_type(img: &Mat, colours: Vec<Colour>) -> Result<Vec<Line>, CvError> {
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

  let mut lines: Vec<Line> = Vec::new();

  for colour_enum in colours {
    let colour: &[[u8; 3]; 2] = get_colour(colour_enum);

    // Extract the colours
    let colour_low = Mat::from_slice::<u8>(&colour[0])?;
    let colour_high = Mat::from_slice::<u8>(&colour[1])?;

    let mut colour_img = Mat::default();
    in_range(&hsv_img, &colour_low, &colour_high, &mut colour_img)?;

    #[cfg(debug_assertions)]
    {
      match colour_enum {
        Colour::Yellow => {
          opencv::highgui::imshow("yellow", &colour_img)?;
        }
        Colour::White => {
          opencv::highgui::imshow("white", &colour_img)?;
        }
        _ => (),
      };
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
