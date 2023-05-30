use crate::{cv_error::CvError, image::convert_to_rgb, line::Colour, line::Line};
use opencv::{
  core::{Mat, Point, Scalar},
  highgui::{imshow, wait_key},
  imgproc::{line, LINE_AA},
  prelude::MatTraitConst,
};

pub fn draw_lines(img: &mut Mat, lines: &Vec<Line>) {
  if let Ok(img) = &mut convert_to_rgb(img) {
    let img_width = img.cols();
    let img_height = img.rows();

    for l in lines {
      // Truncation here is fine (and needed) as we are just drawing pixels on the screen.
      #[allow(clippy::cast_possible_truncation)]
      let start_point = Point::new(l.start.x as i32 * img_width, l.start.y as i32 * img_height);
      #[allow(clippy::cast_possible_truncation)]
      let end_point = Point::new(l.end.x as i32 * img_width, l.end.y as i32 * img_height);

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
