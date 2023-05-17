use cv_error::CvError;
use image_part::ImagePart;
use opencv::{
  core::{in_range, Point, Scalar, Vec4f, Vector},
  imgproc::{line, cvt_color, LINE_AA, COLOR_BGR2HSV},
  ximgproc::FastLineDetector, Result,
  // highgui::{imshow, wait_key},
  prelude::{MatTrait, MatTraitConstManual},
  ximgproc::create_fast_line_detector,
};
use line::{
  Line, HSV_WHITE, HSV_YELLOW, Colour, Pos,
};
use Colour::{White, Yellow};

pub use opencv::prelude::Mat;

pub mod cv_error;
pub mod image_part;
pub mod line;

/// Crops the image in half to reduce needed computation.
///
/// The specified image part is the one ***kept*** in the resulting image.
pub fn crop_image(img: &mut Mat, keep: ImagePart) -> Result<Mat, CvError> {
  let half_height = img.size().map_err(|e| CvError::Other(e.message))?.height / 2;

  let crop = match keep {
    ImagePart::Top => img.adjust_roi(0, -half_height, 0, 0),
    ImagePart::Bottom => img.adjust_roi(-half_height, 0, 0, 0),
  }
  .map_err(|e| CvError::Other(e.message))?;

  Ok(crop)
}

fn get_lines(img: &Mat, colour: Colour) -> Result<Vec<Line>, CvError> {
  let mut fast_line_detector = create_fast_line_detector(20, 1.41, 150.0, 350.0, 3, true)
    .map_err(|_e| CvError::LineDetectorCreation)?;

  let mut lines = Vector::<Vec4f>::default();

  fast_line_detector
    .detect(&img, &mut lines)
    .map_err(|_e| CvError::NoLinesDetected)?;

  let mut line_vec: Vec<Line> = Vec::default();

  for line in lines {
    line_vec.push(Line{colour, pos1: Pos{x: line[0], y: line[1]}, pos2: Pos{x: line[2], y: line[3]}});
  }
  Ok(line_vec)
}

fn get_colour(colour: Colour) -> &'static [[u8; 3]; 2] {
  match colour {
    White => HSV_WHITE,
    Yellow => HSV_YELLOW
  }
}

pub fn detect_line_type(img: &Mat, colours: Vec<Colour>) -> Result<Vec<Line>, CvError> {
  let mut copy_img = Mat::copy(img).expect("copy image");
  let cropped_img = crop_image(&mut copy_img, ImagePart::Bottom).expect("crop image");
  let mut hsv_img = Mat::default();
  cvt_color(&cropped_img, &mut hsv_img, COLOR_BGR2HSV, 0).expect("convert colour"); 

  let mut lines: Vec<Line> = Vec::new();

  for colour_enum in colours {
    let colour: &[[u8; 3]; 2] = get_colour(colour_enum);

    // Extract the colours
    let colour_low = Mat::from_slice::<u8>(&colour[0]).expect("get low colour");
    let colour_high = Mat::from_slice::<u8>(&colour[1]).expect("get high colour");

    let mut colour_img = Mat::default();

    in_range(&hsv_img, &colour_low, &colour_high, &mut colour_img).expect("colour in range");

    // imshow("test", &colour_img).expect("open window");
    // let _res = wait_key(0).expect("keep window open");

    // Get the lines of this colour
    let mut new_lines = get_lines(&colour_img, colour_enum).expect("get lines with colour");

    lines.append(&mut new_lines);
  }
  
  Ok(lines)
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
  let now = std::time::Instant::now();

  let colours = vec![Yellow, White];
  let lines = detect_line_type(&img, colours).expect("process image");

  println!("{:?}", now.elapsed());

  let mut draw_img = crop_image(&mut img, ImagePart::Bottom).expect("crop image");

  // Lines contain a list of 4d vectors that, as stated in `FastLineDetector::detect`, holds the
  // values for `x1, y1, x2, y2`.
  for l in lines {

    // Truncation here is fine (and needed) as we are just drawing pixels on the screen.
    #[allow(clippy::cast_possible_truncation)]
    let start_point = Point::new(l.pos1.x as i32, l.pos1.y as i32);
    #[allow(clippy::cast_possible_truncation)]
    let end_point = Point::new(l.pos2.x as i32, l.pos2.y as i32);

    // OpenCV uses BGR (not RBG) so this is actually red (not that it matters since its greyscale).
    let colour;

    match l.colour {
      Yellow => colour = Scalar::new(0.0, 255.0, 255.0, 0.0),
      White => colour = Scalar::new(255.0, 255.0, 255.0, 0.0),
    }

    line(
      &mut draw_img,
      start_point,
      end_point,
      colour,
      5,
      LINE_AA,
      0,
    )
    .map_err(|_e| CvError::Drawing).expect("draw");
  }

  Ok(draw_img)
}

/// Performs line detection and shows the image in a window.
pub fn show_in_window(img: &Mat) {
  if let Ok(lines) = process_image(img.clone()) {
    opencv::highgui::imshow("img_rgb", &lines).expect("open window");
    let _res = opencv::highgui::wait_key(0).expect("keep window open");
  }
}
