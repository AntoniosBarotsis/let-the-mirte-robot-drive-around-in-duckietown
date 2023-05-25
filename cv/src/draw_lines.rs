use crate::{convert_to_rgb, cv_error::CvError, line::Colour, line::Line};
use opencv::{
  core::{Mat, Point, Scalar},
  highgui::{imshow, wait_key},
  imgcodecs::{imread, IMREAD_UNCHANGED},
  imgproc::{line, LINE_AA},
};

// create function that reads image from path
pub fn read_image(path: &str) -> Result<Mat, CvError> {
  imread(path, IMREAD_UNCHANGED).map_err(|e| CvError::IoError(e.to_string()))
}

pub fn draw_lines(img: &mut Mat, lines: &Vec<Line>) {
  if let Ok(img) = &mut convert_to_rgb(img) {
    for l in lines {
      // Truncation here is fine (and needed) as we are just drawing pixels on the screen.
      #[allow(clippy::cast_possible_truncation)]
      let start_point = Point::new(l.start.x as i32, l.start.y as i32);
      #[allow(clippy::cast_possible_truncation)]
      let end_point = Point::new(l.end.x as i32, l.end.y as i32);

      // OpenCV uses BGR (not RBG).
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

      line(img, start_point, end_point, colour, 5, LINE_AA, 0)
        .map_err(|_e| CvError::Drawing)
        .expect("draw");
    }
    imshow("test", img).expect("open window");
    let _res = wait_key(0).expect("keep window open");
  } else {
    eprintln!("Could not convert colour of image");
  }
}
