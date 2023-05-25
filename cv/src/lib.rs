use cv_error::CvError;
use image_part::ImagePart;
use line::{Colour, Line, Pos, HSV_GREEN, HSV_WHITE, HSV_YELLOW};
use opencv::{
  core::{
    convert_scale_abs, in_range, Point, Scalar, Size_, Vec4f, Vector, CV_32SC1, CV_MAT_DEPTH_MASK,
  },
  imgproc::{
    calc_hist, cvt_color, line, resize, COLOR_BGR2HSV, COLOR_BGR2RGB, COLOR_RGB2GRAY,
    COLOR_RGB2HSV, INTER_AREA, LINE_AA,
  },
  prelude::{MatTrait, MatTraitConst, MatTraitConstManual},
  ximgproc::create_fast_line_detector,
  ximgproc::FastLineDetector,
  Result,
};

pub use opencv::prelude::Mat;

pub mod cv_error;
pub mod draw_lines;
pub mod image_part;
pub mod line;

/// Crops the image in half to reduce needed computation.
///
/// The specified image part is the one ***kept*** in the resulting image.
pub fn crop_image(img: &mut Mat, keep: ImagePart) -> Result<Mat, CvError> {
  let new_height = img.size()?.height * 60 / 100;

  let crop = match keep {
    ImagePart::Top => img.adjust_roi(0, -new_height, 0, 0),
    ImagePart::Bottom => img.adjust_roi(-new_height, 0, 0, 0),
  }?;

  Ok(crop)
}

fn get_lines(img: &Mat, colour: Colour, half_height: f32) -> Result<Vec<Line>, CvError> {
  let mut fast_line_detector = create_fast_line_detector(20, 1.41, 150.0, 350.0, 3, true)
    .map_err(|_e| CvError::LineDetectorCreation)?;

  let mut lines = Vector::<Vec4f>::default();

  fast_line_detector
    .detect(&img, &mut lines)
    .map_err(|_e| CvError::NoLinesDetected)?;

  let mut line_vec: Vec<Line> = Vec::default();

  for line in lines {
    line_vec.push(Line::new(
      colour,
      Pos::new(line[0], line[1] + half_height),
      Pos::new(line[2], line[3] + half_height),
    ));
  }
  Ok(line_vec)
}

fn scale_contrast(img: &Mat) -> Result<Mat, CvError> {
  // Change this factor for the contrast clipping
  const CLIP_FACTOR: f32 = 100.0;

  println!("img depth {}", Mat::depth(img) & CV_MAT_DEPTH_MASK);
  println!("img channels {}", Mat::channels(img));

  let mut gray_img = Mat::default();
  cvt_color(&img, &mut gray_img, COLOR_RGB2GRAY, 0)?;

  println!("img depth {}", Mat::depth(&gray_img) & CV_MAT_DEPTH_MASK);
  println!("img channels {}", Mat::channels(&gray_img));

  let mut type_img = Mat::default();
  gray_img.convert_to(&mut type_img, CV_32SC1, 1.0, 0.0)?;

  let mut hist = Mat::default();

  let mut range = opencv::core::Vector::<f32>::from_elem(0.0, 1);
  range.push(256.0);

  // TODO: Check why calc_hist returns an error
  calc_hist(
    &opencv::core::Vector::<Mat>::from_elem(gray_img, 1),
    &opencv::core::Vector::<i32>::from_elem(0, 1),
    &Mat::default(),
    &mut hist,
    &opencv::core::Vector::<i32>::from_elem(256, 1),
    &range,
    false,
  )?;

  // println!("Mat data type {:?}", img.data_typed()?);
  println!("{:?}", hist.rows());

  let hist_len = hist.rows() as usize;

  let mut acc: [f32; 256] = [0.0; 256];
  acc[0] = *hist.at::<f32>(0)?;

  println!("{}", acc[0]);
  for n in 1..hist_len {
    // TODO: check if we need -1 or not
    acc[n] = acc[n - 1] + *hist.at::<f32>(i32::try_from(n).expect("convert"))?;
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

  #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
  let a = 255.0 / (max_gray as f32 - min_gray as f32);
  #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
  let b = -(min_gray as f32) * a;

  let mut contrast_img = Mat::default();

  #[allow(clippy::cast_lossless)]
  convert_scale_abs(&img, &mut contrast_img, a as f64, b as f64)?;

  println!("bla");

  Ok(contrast_img)
}

fn get_colour(colour: Colour) -> &'static [[u8; 3]; 2] {
  match colour {
    Colour::White => HSV_WHITE,
    Colour::Yellow => HSV_YELLOW,
    Colour::Green => HSV_GREEN,
    colour => panic!("No HSV constants defined for {colour:?}!"),
  }
}

pub fn convert_to_rgb(img: &Mat) -> Result<Mat, CvError> {
  let mut rgb_img = Mat::default();
  cvt_color(&img, &mut rgb_img, COLOR_BGR2RGB, 0)?;
  Ok(rgb_img)
}

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

pub fn detect_line_type(img: &Mat, colours: Vec<Colour>) -> Result<Vec<Line>, CvError> {
  let mut copy_img = Mat::copy(img)?;

  let cropped_img = crop_image(&mut copy_img, ImagePart::Bottom)?;

  let img_height = img.rows() - cropped_img.rows();

  // Contrast stretching
  let contrast_img = scale_contrast(&cropped_img)?;

  let rgb_img = convert_to_rgb(&contrast_img)?;
  opencv::highgui::imshow("contrast", &rgb_img).expect("open window");
  // let _res = opencv::highgui::wait_key(0).expect("keep window open");

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

    match colour_enum {
      Colour::Yellow => {
        opencv::highgui::imshow("yellow", &colour_img).expect("open window");
        // let _res = opencv::highgui::wait_key(0).expect("keep window open");
      }
      Colour::White => {
        opencv::highgui::imshow("white", &colour_img).expect("open window");
        // let _res = opencv::highgui::wait_key(0).expect("keep window open");
      }
      _ => (),
    };

    // Get the lines of this colour
    // Casting an i32 to an f32 is fine, as the image height is realistically never going to exceed
    // the maximum value of an f32.

    // This takes about 1/6 of the time for 2 colours
    #[allow(clippy::cast_precision_loss)]
    let mut new_lines = get_lines(&colour_img, colour_enum, img_height as f32)?;

    lines.append(&mut new_lines);
  }
  Ok(lines)
}

/// Detects lines in the input image, plots them and returns the result.
///
/// The image can be in colour.
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
///   imgcodecs::{self, imread, imwrite, IMREAD_UNCHANGED},
/// };
///
/// // We can import the image in colour because the FastLineDetector requires it.
/// let img = imread("../assets/input_1.jpg", IMREAD_UNCHANGED).expect("open image");

/// let output = process_image(img).unwrap();

/// // Save output image.
/// let saved = imwrite("../assets/output.jpg", &output, &Vector::default())
///   .map_err(|e| CvError::IoError(e.message)).unwrap();
///
/// // Make sure that the image was saved correctly
/// assert!(saved);
/// ```
pub fn process_image(mut img: Mat) -> Result<Mat, CvError> {
  let now = std::time::Instant::now();

  let colours = vec![Colour::Yellow, Colour::White];
  let lines = detect_line_type(&img, colours)?;

  println!("{:?}", now.elapsed());

  let mut draw_img = crop_image(&mut img, ImagePart::Bottom)?;

  // Lines contain a list of 4d vectors that, as stated in `FastLineDetector::detect`, holds the
  // values for `x1, y1, x2, y2`.
  for l in lines {
    // Truncation here is fine (and needed) as we are just drawing pixels on the screen.
    #[allow(clippy::cast_possible_truncation)]
    let start_point = Point::new(l.start.x as i32, l.start.y as i32);
    #[allow(clippy::cast_possible_truncation)]
    let end_point = Point::new(l.end.x as i32, l.end.y as i32);

    // OpenCV uses BGR (not RBG) so this is actually red (not that it matters since its greyscale).
    let colour = match l.colour {
      Colour::Red => Scalar::new(0.0, 0.0, 255.0, 0.0),
      Colour::Orange => Scalar::new(0.0, 128.0, 255.0, 0.0),
      Colour::Yellow => Scalar::new(0.0, 255.0, 255.0, 0.0),
      Colour::Green => Scalar::new(0.0, 255.0, 0.0, 0.0),
      Colour::Blue => Scalar::new(255.0, 0.0, 0.0, 0.0),
      Colour::Purple => Scalar::new(255.0, 0.0, 255.0, 0.0),
      Colour::Black => Scalar::new(0.0, 0.0, 0.0, 0.0),
      Colour::White => Scalar::new(255.0, 255.0, 255.0, 0.0),
    };

    line(&mut draw_img, start_point, end_point, colour, 5, LINE_AA, 0)
      .map_err(|_e| CvError::Drawing)
      .expect("Failed to draw lines on image");
  }

  Ok(draw_img)
}

/// Performs line detection and shows the image in a window.
///
/// # Panics
///
/// Panics if image colour can't be converted.
pub fn show_in_window(img: &Mat) {
  // if let Ok(img_rgb) = convert_to_rgb(img) {
  opencv::highgui::imshow("img_rgb", &img).expect("open window");
  let _res = opencv::highgui::wait_key(0).expect("keep window open");
  // } else {
  //   panic!("Failed to convert colour of image");
  // }
}
