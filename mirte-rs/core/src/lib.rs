pub mod detection;

use common::{
  debug, edebug,
  structs::{colour::ColourEnum, threshold::Threshold},
};
use cv::{
  detect_lines::detect_line_type, image::downscale_enhance_hsv, object::detect_obstacles, Mat,
};
use detection::{detect_lane, detect_stop_line};
use ros::publishers::RosBgPublisher;
use std::collections::HashMap;

#[cfg(debug_assertions)]
use std::time::Instant;

/// Does all required processing on a given image and publishes the data on the correct topics.
/// This consists of detecting lines, the lane, stop lines, and obstacles. This function does not
/// fail, since we want the robot to keep running even if a single image may have an error.
/// If run in debug mode, it also prints the time each process took, and any debug information
/// if the process failed in any way.
///
/// * `mat` - The HSV image in which the lines need to be detected
/// * `thresholds` - The thresholds for each colour. These must at least contain Yellow, White and
/// Red
///
/// # Usage
/// To see how to use this method, refer to `main.rs`, where this function is used.
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
        use ros::mirte_msgs::LineSegment;
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

#[cfg(test)]
mod test {
  use super::process_mat;
  use common::structs::{colour::ColourEnum, threshold::Threshold};
  use common::{
    geometry_msgs::{Point, Vector3},
    mirte_msgs::Line,
  };
  use cv::image::read_image;
  //use rosrust_msg::std_msgs as msgs;
  use rostest_macro::ros_test;
  use std::collections::HashMap;

  fn default_process_mat() {
    let thresholds: HashMap<ColourEnum, Threshold> =
      [ColourEnum::Red, ColourEnum::Yellow, ColourEnum::White]
        .iter()
        .map(|&colour| (colour, Threshold::by_colour(colour)))
        .collect();
    let path = "../assets/test_images/test_image_1.png";
    let img = read_image(path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));
    process_mat(&img, &thresholds);
  }

  #[ros_test]
  fn test_process_mat() {
    // Init topics
    let stop_line_topic = rostest::Topic::<Line>::create("/test/stop_line");

    // Create node
    rostest::instantiate_node(default_process_mat);

    // Publish message
    //let message = msgs::String {
    //  data: "Hello World".to_string(),
    //};
    //line_data.ros_publish(message);

    // Assert response
    let expected = Line::new(Point::new(1.0, 0.0), Vector3::new(0.0, 0.0));

    //stop_line_topic.assert_message(expected);
  }
}
