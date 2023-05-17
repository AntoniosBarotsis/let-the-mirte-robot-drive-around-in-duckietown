use opencv::{
  core::{Mat, Point, Scalar},
  highgui::{imshow, wait_key},
  imgcodecs::{imread, IMREAD_UNCHANGED},
  imgproc::{line, LINE_AA},
};

use crate::{
  cv_error::CvError,
  line::Colour::{White, Yellow},
  line::Line,
};

// create function that reads image from path
pub fn read_image(path: &str) -> Result<Mat, CvError> {
  imread(path, IMREAD_UNCHANGED).map_err(|e| CvError::IoError(e.to_string()))
}

pub fn draw_lines(mut img: &mut Mat, lines: Vec<Line>) {
  for l in lines {
    // Truncation here is fine (and needed) as we are just drawing pixels on the screen.
    #[allow(clippy::cast_possible_truncation)]
    let start_point = Point::new(l.pos1.x as i32, l.pos1.y as i32);
    #[allow(clippy::cast_possible_truncation)]
    let end_point = Point::new(l.pos2.x as i32, l.pos2.y as i32);

    // OpenCV uses BGR (not RBG).
    let colour = match l.colour {
      Yellow => Scalar::new(0.0, 255.0, 255.0, 0.0),
      White => Scalar::new(255.0, 255.0, 255.0, 0.0),
    };

    line(&mut img, start_point, end_point, colour, 5, LINE_AA, 0)
      .map_err(|_e| CvError::Drawing)
      .expect("draw");
  }

  imshow("test", img).expect("open window");
  let _res = wait_key(0).expect("keep window open");
}
