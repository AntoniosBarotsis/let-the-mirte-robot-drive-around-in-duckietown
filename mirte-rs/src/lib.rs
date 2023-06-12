pub mod detect_lane;
pub mod mirte_error;

use std::time::Instant;

use cv::{
  detect_lines::detect_line_type,
  image::{downscale, downscale_enhance_hsv},
  line::{
    Colour::{self},
    Threshold,
  },
  object::detect_obstacles,
  Mat,
};
use detect_lane::detect_lane;
use mirte_error::MirteError;
use ros::{process_ros_image_one, publishers::RosBgPublisher, CvImage};
use std::collections::HashMap;

#[cfg(debug_assertions)]
use ::cv::{
  draw_lines::draw_lines,
  line::Colour::{Blue, Green, Red},
};

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
  mat: &Mat,
  thresholds: &HashMap<Colour, Threshold, S>,
) {
  let time_total = Instant::now();

  // Converting to usable image
  let time_hsv = Instant::now();
  if let Ok(hsv_img) = downscale_enhance_hsv(mat) {
    println!("converting to hsv: {:?}", time_hsv.elapsed());

    // Creating ROS publisher
    let publisher = RosBgPublisher::get_or_create();

    // Detecting lines
    let colours = vec![cv::line::Colour::Yellow, cv::line::Colour::White];
    let time_detecting_lines = Instant::now();
    if let Ok(lines) = detect_line_type(&hsv_img, thresholds, colours) {
      println!("detecting lines: {:?}", time_detecting_lines.elapsed());
      publisher.publish_line_segment(lines.clone());

      // Detecting lane
      let time_detect_lane = Instant::now();
      let lane = detect_lane(&lines);
      println!("detecting lane: {:?}", time_detect_lane.elapsed());
      publisher.publish_lane(lane);

      // Draw lines and lane in debug
      #[cfg(debug_assertions)]
      {
        let all_lines = [lines, lane.get_coloured_segments(Green, Blue, Red)].concat();
        if let Ok(resized) = downscale(mat) {
          draw_lines(&resized, &all_lines);
        } else {
          eprintln!("could not downscale image for drawing the lines");
        }
      }
    } else {
      eprintln!("Could not detect lines");
    }

    let time_detecting_obstacles = Instant::now();
    if let Ok(obstacles) = detect_obstacles(&hsv_img) {
      println!(
        "detecting obstacles: {:?}",
        time_detecting_obstacles.elapsed()
      );
      publisher.publish_obstacle(obstacles);
    } else {
      eprintln!("Could not detect obstacles");
    }
  } else {
    eprintln!("Could not downscale, enhance or convert to hsv");
  }

  println!("total: {:?}", time_total.elapsed());
}
