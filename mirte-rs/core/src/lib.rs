pub mod detection;
pub mod mirte_error;

#[cfg(debug_assertions)]
use std::time::Instant;

use common::{
  debug, edebug,
  structs::{colour::ColourEnum, threshold::Threshold},
};
use cv::{
  detect_lines::detect_line_type, image::downscale_enhance_hsv, object::detect_obstacles, Mat,
};
use detection::{detect_lane, detect_stop_line};
use mirte_error::MirteError;
use ros::{process_ros_image_one, publishers::RosBgPublisher, CvImage};
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
  mat: &Mat,
  thresholds: &HashMap<ColourEnum, Threshold, S>,
) {
  #[cfg(debug_assertions)]
  let time_total = Instant::now();

  // Converting to usable image
  #[cfg(debug_assertions)]
  let time_hsv = Instant::now();
  if let Ok(hsv_img) = downscale_enhance_hsv(mat) {
    debug!("converting to hsv: {:?}", time_hsv.elapsed());

    // Creating ROS publisher
    let publisher = RosBgPublisher::get_or_create();

    // Detecting lines
    let colours = vec![ColourEnum::Yellow, ColourEnum::White, ColourEnum::Red];
    #[cfg(debug_assertions)]
    let time_detecting_lines = Instant::now();
    if let Ok(lines) = detect_line_type(&hsv_img, thresholds, colours) {
      debug!("detecting lines: {:?}", time_detecting_lines.elapsed());
      publisher.publish_line_segment(lines.clone());

      // Detecting lane
      #[cfg(debug_assertions)]
      let time_detect_lane = Instant::now();
      let lane = detect_lane(&lines);
      debug!("detecting lane: {:?}", time_detect_lane.elapsed());
      publisher.publish_lane(lane);

      // Detecting stop line
      #[cfg(debug_assertions)]
      let time_detect_stop_line = Instant::now();
      let stop_line = detect_stop_line(&lines);
      debug!("detecting stop line: {:?}", time_detect_stop_line.elapsed());
      publisher.publish_stop_line(stop_line);

      // Draw lines, lane and stop line in debug
      #[cfg(debug_assertions)]
      {
        use cv::{draw_lines::draw_lines, image::downscale};
        use ros::mirte_duckietown_msgs::LineSegment;

        let all_lines = [
          lines,
          lane.get_coloured_segments(ColourEnum::Green, ColourEnum::Orange, ColourEnum::Black),
          vec![LineSegment::from_line(stop_line, ColourEnum::Purple)],
        ]
        .concat();
        if let Ok(resized) = downscale(mat) {
          draw_lines(&resized, &all_lines);
        } else {
          edebug!("could not downscale image for drawing the lines");
        }
      }
    } else {
      edebug!("Could not detect lines");
    }

    #[cfg(debug_assertions)]
    let time_detecting_obstacles = Instant::now();
    if let Ok(obstacles) = detect_obstacles(&hsv_img) {
      debug!(
        "detecting obstacles: {:?}",
        time_detecting_obstacles.elapsed()
      );
      publisher.publish_obstacle(obstacles);
    } else {
      edebug!("Could not detect obstacles");
    }
  } else {
    edebug!("Could not downscale, enhance or convert to hsv");
  }

  debug!("total: {:?}", time_total.elapsed());
}
