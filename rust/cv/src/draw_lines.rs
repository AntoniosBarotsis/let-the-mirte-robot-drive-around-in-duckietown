use crate::{cv_error::CvError, image::convert_to_rgb};
use common::edebug;
use common::{mirte_duckietown_msgs::LineSegment, structs::colour::ColourEnum};
use opencv::{
  core::{Mat, Point, Scalar},
  highgui::{imshow, wait_key},
  imgproc::{line, LINE_AA},
  prelude::MatTraitConst,
};

pub fn draw_lines(img: &Mat, lines: &Vec<LineSegment>) {
  if let Ok(img) = &mut convert_to_rgb(img) {
    #[allow(clippy::cast_precision_loss)]
    let img_width = f64::from(img.cols());
    #[allow(clippy::cast_precision_loss)]
    let img_height = f64::from(img.rows());

    for l in lines {
      // Truncation here is fine (and needed) as we are just drawing pixels on the screen.
      #[allow(clippy::cast_possible_truncation)]
      let start_point = Point::new(
        (l.start.x * img_width) as i32,
        (l.start.y * img_height) as i32,
      );
      #[allow(clippy::cast_possible_truncation)]
      let end_point = Point::new((l.end.x * img_width) as i32, (l.end.y * img_height) as i32);

      // OpenCV uses BGR (not RBG).
      let colour = match l.colour.into() {
        ColourEnum::Red => Scalar::new(0.0, 0.0, 255.0, 0.0),
        ColourEnum::Orange => Scalar::new(0.0, 128.0, 255.0, 0.0),
        ColourEnum::Yellow => Scalar::new(0.0, 255.0, 255.0, 0.0),
        ColourEnum::Green => Scalar::new(0.0, 255.0, 0.0, 0.0),
        ColourEnum::Blue => Scalar::new(255.0, 0.0, 0.0, 0.0),
        ColourEnum::Purple => Scalar::new(255.0, 0.0, 255.0, 0.0),
        ColourEnum::Black => Scalar::new(0.0, 0.0, 0.0, 0.0),
        ColourEnum::White => Scalar::new(255.0, 255.0, 255.0, 0.0),
      };

      #[allow(clippy::expect_used)]
      line(img, start_point, end_point, colour, 5, LINE_AA, 0)
        .map_err(|_e| CvError::Drawing)
        .expect("draw");
    }
    #[allow(clippy::expect_used)]
    {
      imshow("test", img).expect("open window");
      let _res = wait_key(0).expect("keep window open");
    }
  } else {
    edebug!("Could not convert colour of image");
  }
}