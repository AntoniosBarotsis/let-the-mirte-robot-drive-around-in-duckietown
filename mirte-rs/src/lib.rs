pub mod detection;
pub mod mirte_error;

use std::time::Instant;

use cv::{
  detect_lines::detect_line_type, draw_lines::draw_lines, image::downscale, line::Threshold, Mat,
};
use detection::{detect_lane, detect_stop_line};
use mirte_error::MirteError;
use ros::{
  mirte_msgs::{Colour, LineSegment},
  process_ros_image_one,
  publishers::RosBgPublisher,
  CvImage,
};
use std::collections::HashMap;

#[allow(clippy::unwrap_used, clippy::expect_used, clippy::missing_panics_doc)]
pub fn get_image() -> Result<Mat, MirteError> {
  let img = process_ros_image_one()?;

  let mat = CvImage::from_imgmsg(img).unwrap().as_cvmat().unwrap();

  // This clone here, although seemingly useless, fixes a weird bug that causes artifacts to
  // appear during the color conversion. For more details, refer to:
  // https://github.com/twistedfall/opencv-rust/issues/277
  #[allow(clippy::redundant_clone)]
  Ok(mat.clone())
}

pub fn process_mat<S: std::hash::BuildHasher>(
  mat: Mat,
  thresholds: &HashMap<Colour, Threshold, S>,
) {
  let time_total = Instant::now();
  let time_1 = Instant::now();

  let mut resized = downscale(&mat).unwrap_or(mat);
  println!("resizing: {:?}", time_1.elapsed());

  let colours = vec![
    Colour {
      type_: Colour::YELLOW,
    },
    Colour {
      type_: Colour::WHITE,
    },
    Colour { type_: Colour::RED },
  ];

  let time_2 = Instant::now();

  if let Ok(lines) = detect_line_type(&resized, thresholds, colours) {
    let publisher = RosBgPublisher::get_or_create();
    publisher.publish_line_segment(lines.clone());

    println!("detecting lines: {:?}", time_2.elapsed());

    let time_3 = Instant::now();
    let lane = detect_lane(&lines);

    publisher.publish_lane(lane);

    println!("detecting lane: {:?}", time_3.elapsed());

    let time_4 = Instant::now();
    let stop_line = detect_stop_line(&lines);

    publisher.publish_stop_line(stop_line);

    println!("detecting stop line: {:?}", time_4.elapsed());

    let all_lines = [
      lines,
      lane.get_coloured_segments(
        Colour {
          type_: Colour::GREEN,
        },
        Colour {
          type_: Colour::ORANGE,
        },
        Colour {
          type_: Colour::BLACK,
        },
      ),
      vec![LineSegment::from_line(
        stop_line,
        Colour {
          type_: Colour::PURPLE,
        },
      )],
    ]
    .concat();
    draw_lines(&mut resized, &all_lines);
  } else {
    eprintln!("Could not detect lines");
  }

  println!("total: {:?}", time_total.elapsed());
}
