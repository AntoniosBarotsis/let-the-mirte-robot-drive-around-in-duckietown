use cv_error::CvError;
use image_part::ImagePart;
use line::{HSV_WHITE, HSV_YELLOW};
use opencv::{
  core::{convert_scale_abs, in_range, Size, Size_, Vec4f, Vector, CV_32SC1},
  imgproc::{
    calc_hist, cvt_color, resize, COLOR_BGR2RGB, COLOR_RGB2GRAY, COLOR_RGB2HSV, INTER_AREA,
  },
  prelude::{MatTrait, MatTraitConst, MatTraitConstManual},
  ximgproc::create_fast_line_detector,
  ximgproc::FastLineDetector,
  Result,
};

pub use line::{Colour, Line, Pos};
pub use opencv::prelude::Mat;

pub mod cv_error;
pub mod draw_lines;
pub mod image_part;
pub mod line;

/// Crops the image to reduce redundant information. It will split it into two part. A top part and gain bottom part.
///
/// * `img` - The image that needs to be cropped
/// * `keep` - Which part of the image you want to keep. There are two options. `ImagePart::Top` give the top part and `ImagePart::Bottom` gives the bottom
///
/// Returns gain result with the specificed part as `Mat`
pub fn crop_image(img: &mut Mat, keep: ImagePart) -> Result<Mat, CvError> {
  let new_height = img.size()?.height * 3 / 5;

  let crop = match keep {
    ImagePart::Top => img.adjust_roi(0, -new_height, 0, 0),
    ImagePart::Bottom => img.adjust_roi(-new_height, 0, 0, 0),
  }?;

  Ok(crop)
}

/// Finds lines in the image with gain specific colour using the `fast_line_detector` from `openCV`
///
/// * `img` - The images of which the lines need to be detected
/// * `colour` - The colour of which you want to detect the lanes
/// * `line_offset` - The Y offset of where to draw the lines
///
/// Returns gain result of gain vector of lines found on the image with the specified colour
fn get_lines(
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

/// Enhances the contrast of the image using histogram stretching
///
/// * `img` - The image which contrast needs to be enhanced
///
/// Returns gain result of image which enhanced contrast as `Mat`
fn enhance_contrast(img: &Mat) -> Result<Mat, CvError> {
  let mut gray_img = Mat::default();
  cvt_color(&img, &mut gray_img, COLOR_RGB2GRAY, 0)?;

  let mut type_img = Mat::default();
  gray_img.convert_to(&mut type_img, CV_32SC1, 1.0, 0.0)?;

  let mut hist = Mat::default();

  let mut range = opencv::core::Vector::<f32>::from_elem(0.0, 1);
  range.push(256.0);

  calc_hist(
    &opencv::core::Vector::<Mat>::from_elem(gray_img, 1),
    &opencv::core::Vector::<i32>::from_elem(0, 1),
    &Mat::default(),
    &mut hist,
    &opencv::core::Vector::<i32>::from_elem(256, 1),
    &range,
    false,
  )?;

  let (gain, bias) = calc_gain_bias(&hist)?;

  let mut contrast_img = Mat::default();

  #[allow(clippy::cast_lossless)]
  convert_scale_abs(&img, &mut contrast_img, gain as f64, bias as f64)?;

  Ok(contrast_img)
}

/// Calculates the gain and bias for contrast enhancing
///
/// * `hist` - The histogram to calculate the gain and bias from
///
/// Returns gain and bias
fn calc_gain_bias(hist: &Mat) -> Result<(f32, f32), CvError> {
  // Change this factor for the contrast clipping
  const CLIP_FACTOR: f32 = 100.0;

  #[allow(clippy::cast_sign_loss)]
  let hist_len = hist.rows() as usize;

  let mut acc: [f32; 256] = [0.0; 256];
  acc[0] = *hist.at::<f32>(0)?;

  for n in 1..hist_len {
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    {
      acc[n] = acc[n - 1] + *hist.at::<f32>(n as i32)?;
    }
  }

  let max = acc[acc.len() - 1];
  let clip = max / CLIP_FACTOR;

  let mut min_gray = 0;
  while acc[min_gray] < clip {
    min_gray += 1;
  }

  let mut max_gray = acc.len() - 1;
  while acc[max_gray] >= (max - clip) {
    max_gray -= 1;
  }

  #[allow(clippy::cast_precision_loss)]
  let gain = 255.0 / (max_gray as f32 - min_gray as f32);
  #[allow(clippy::cast_precision_loss)]
  let bias = -(min_gray as f32) * gain;

  Ok((gain, bias))
}

/// given a colour type it will return the lower and upper bound of the range of that colour in HSV
///
/// * `colour` - The colour of which the colour range needs to be extracted
///
/// Return an 2d-array with the lower bound on index 0 and upper bound on index 1
fn get_colour(colour: Colour) -> &'static [[u8; 3]; 2] {
  match colour {
    Colour::White => HSV_WHITE,
    Colour::Yellow => HSV_YELLOW,
    colour => panic!("No HSV constants defined for {colour:?}!"),
  }
}

/// converts a given image with a BGR colour format to one with a RGB colour format
///
/// * `img` - The image who's colour format needs to be chanced
///
/// Returns a result of the image with the RGB colour format as a `Mat`
pub fn convert_to_rgb(img: &Mat) -> Result<Mat, CvError> {
  let mut rgb_img = Mat::default();
  cvt_color(&img, &mut rgb_img, COLOR_BGR2RGB, 0)?;
  Ok(rgb_img)
}

/// Given an image it will downscale that image to a width of 320 and height of 240
///
/// * `img` - The image that needs to be downscaled
///
/// Returns a result with the now downscaled image as a `Mat`
pub fn downscale(img: &Mat) -> Result<Mat, CvError> {
  let mut resized = Mat::default();

  resize(
    &img,
    &mut resized,
    Size_ {
      width: 320,  // 320
      height: 240, // 240
    },
    0.0,
    0.0,
    INTER_AREA,
  )?;

  Ok(resized)
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

  // debug
  // let rgb_img = convert_to_rgb(&contrast_img)?;
  // opencv::highgui::imshow("contrast", &rgb_img).expect("open window");

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

    // Debug
    // match colour_enum {
    //   Colour::Yellow => {
    //     opencv::highgui::imshow("yellow", &colour_img).expect("open window");
    //   }
    //   Colour::White => {
    //     opencv::highgui::imshow("white", &colour_img).expect("open window");
    //   }
    //   _ => (),
    // };

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

pub fn dbg_mat() -> Result<Mat, CvError> {
  let mat = draw_lines::read_image("./assets/input_1.jpg")?;

  let size = mat.size()?;

  if size.height == 0 || size.width == 0 {
    return Err(CvError::IoError("Error reading image".to_owned()));
  }

  Ok(mat)
}
