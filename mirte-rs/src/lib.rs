pub mod detection;
pub mod mirte_error;

use std::time::Instant;

use cv::{
  detect_lines::detect_line_type,
  draw_lines::draw_lines,
  image::downscale,
  line::{Colour::{Black, Green, Orange, Red, White, Yellow, Purple}, LineSegment},
  Mat,
};
use detection::{detect_lane, detect_stop_line};
use mirte_error::MirteError;
use ros::{process_ros_image_one, publishers::RosBgPublisher, CvImage};

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

pub fn process_mat(mat: Mat) {
  let time_total = Instant::now();
  let time_1 = Instant::now();

  let mut resized = downscale(&mat).unwrap_or(mat);
  println!("resizing: {:?}", time_1.elapsed());

  let colours = vec![Yellow, White, Red];

  let time_2 = Instant::now();

  if let Ok(lines) = detect_line_type(&resized, colours) {
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

    let all_lines = [lines, lane.get_coloured_segments(Green, Orange, Black), vec![LineSegment::from_line(stop_line, Purple)]].concat();
    draw_lines(&mut resized, &all_lines);
  } else {
    eprintln!("Could not detect lines");
  }

  println!("total: {:?}", time_total.elapsed());
}
