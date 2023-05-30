use opencv::{
  core::{convert_scale_abs, Size_, CV_32SC1},
  imgproc::{calc_hist, cvt_color, resize, COLOR_BGR2RGB, COLOR_RGB2GRAY, INTER_AREA},
  prelude::{Mat, MatTrait, MatTraitConst, MatTraitConstManual},
};

use crate::{cv_error::CvError, draw_lines, image_part::ImagePart};

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

/// Enhances the contrast of the image using histogram stretching
///
/// * `img` - The image which contrast needs to be enhanced
///
/// Returns gain result of image which enhanced contrast as `Mat`
///
/// # Example
///
/// ```
/// use opencv::prelude::MatTraitConst;
///
/// use cv::image::{convert_to_gray, dbg_mat, enhance_contrast};
///
/// let mat = dbg_mat("../assets/test_images/test_image_1.png").expect("couldn't get a matrix");
/// let gray_mat = convert_to_gray(&mat).expect("couldn't convert to gray");
/// let diff = *gray_mat.at::<u8>(0).expect("couldn't get first value") - *gray_mat.at::<u8>(3).expect("couldn't get second value");
/// let output_mat = enhance_contrast(&mat).expect("couldn't enhance contrast");
/// let gray_output_mat = convert_to_gray(&output_mat).expect("couldn't covert to gray");
/// let output_diff = *gray_output_mat.at::<u8>(0).expect("couldn't get first value") - *gray_output_mat.at::<u8>(3).expect("couldn't get second value");
///
/// assert!(diff < output_diff);
/// ```
pub fn enhance_contrast(img: &Mat) -> Result<Mat, CvError> {
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

/// converts a gvien image with a RGB colour format to one in grayscale
///
/// * `img` - The image who's colour format needs to be chanced
///
/// Returns a result of the image as grayscale as a `Mat`
pub fn convert_to_gray(img: &Mat) -> Result<Mat, CvError> {
  let mut gray_img = Mat::default();
  cvt_color(&img, &mut gray_img, COLOR_RGB2GRAY, 0)?;
  Ok(gray_img)
}

/// Given an image it will downscale that image to a width of 320 and height of 240
///
/// * `img` - The image that needs to be downscaled
///
/// Returns a result with the now downscaled image as a `Mat`
///
/// # Examples
///
/// ```
/// use opencv::prelude::MatTraitConstManual;
///
/// use cv::image::{dbg_mat, downscale};
///
/// let mat = dbg_mat("../assets/input_1.jpg").expect("couldn't get a matrix as output");
/// let output_mat = downscale(&mat).expect("couldn't downscale the image");
/// let output_size = output_mat.size().expect("couldn't get matrix size");
///
/// assert!(output_size.width == 320 && output_size.height == 240);
/// ```
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

pub fn dbg_mat(path: &str) -> Result<Mat, CvError> {
  let mat = draw_lines::read_image(path)?;

  let size = mat.size()?;

  if size.height == 0 || size.width == 0 {
    return Err(CvError::IoError("Error reading image".to_owned()));
  }

  Ok(mat)
}
