use cv::line::Colour::{White, Yellow};
use cv::{crop_image, cv_error::CvError, detect_line_type};
use std::time::Instant;

use opencv::{
  core::{Point, Scalar},
  highgui::{imshow, wait_key},
  imgcodecs::{imread, IMREAD_UNCHANGED},
  imgproc::{line, LINE_AA},
};

/// Processes the input image from the assets folder and displays it in a window for an easier
/// inspection. The window can also be closed by pressing any button.
fn main() {
  let mut img = imread("./assets/input_real.jpg", IMREAD_UNCHANGED).expect("open image");

  let now = Instant::now();

  let colours = vec![Yellow, White];
  let lines = detect_line_type(&img, colours).expect("process image");

  println!("{:?}", now.elapsed());

  let mut draw_img = crop_image(&mut img, cv::image_part::ImagePart::Bottom).expect("crop image");

  for l in lines {
    // Truncation here is fine (and needed) as we are just drawing pixels on the screen.
    #[allow(clippy::cast_possible_truncation)]
    let start_point = Point::new(l.pos1.x as i32, l.pos1.y as i32);
    #[allow(clippy::cast_possible_truncation)]
    let end_point = Point::new(l.pos2.x as i32, l.pos2.y as i32);

    // OpenCV uses BGR (not RBG) so this is actually red (not that it matters since its greyscale).
    let colour = match l.colour {
      Yellow => Scalar::new(0.0, 255.0, 255.0, 0.0),
      White => Scalar::new(255.0, 255.0, 255.0, 0.0),
    };

    line(&mut draw_img, start_point, end_point, colour, 5, LINE_AA, 0)
      .map_err(|_e| CvError::Drawing)
      .expect("draw");
    }

  imshow("test", &draw_img).expect("open window");
  let _res = wait_key(0).expect("keep window open");
}
