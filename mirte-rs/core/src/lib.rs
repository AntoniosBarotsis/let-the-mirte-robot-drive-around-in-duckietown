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
  publisher: &RosBgPublisher,
) {
  #[cfg(debug_assertions)]
  let time_total = Instant::now();

  // Converting to usable image
  #[cfg(debug_assertions)]
  let time_hsv = Instant::now();
  if let Ok(hsv_img) = downscale_enhance_hsv(mat) {
    debug!("converting to hsv: {:?}", time_hsv.elapsed());

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

#[cfg(test)]
mod test {
  use super::process_mat;
  use common::structs::{colour::ColourEnum, threshold::Threshold};
  use common::{
    geometry_msgs::{Point, Vector3},
    mirte_msgs::{Lane, Line, Object, Obstacle, ObstacleList},
  };
  use cv::image::read_image;
  use ros::publishers::RosBgPublisher;
  //use rosrust_msg::std_msgs as msgs;
  use rostest_macro::ros_test;
  use std::collections::HashMap;

  fn process_lane_image() {
    process_image("../assets/test_images/test_image_lane.jpg");
  }

  fn process_obstacle_image() {
    process_image("../assets/test_images/test_image_obstacles.jpg");
  }

  fn process_image(path: &str) {
    let thresholds: HashMap<ColourEnum, Threshold> =
      [ColourEnum::Red, ColourEnum::Yellow, ColourEnum::White]
        .iter()
        .map(|&colour| (colour, Threshold::by_colour(colour)))
        .collect();
    let publisher = RosBgPublisher::create();
    let img = read_image(path).unwrap_or_else(|_| panic!("Unable to get image from {path}"));
    process_mat(&img, &thresholds, &publisher);
  }

  #[ros_test]
  fn test_detect_lane() {
    // Init topics
    let stop_line_topic = rostest::Topic::<Line>::create("/stop_line");
    let lane_topic = rostest::Topic::<Lane>::create("/lanes");
    let obstacle_topic = rostest::Topic::<ObstacleList>::create("/obstacles");

    // Create node
    rostest::instantiate_node(process_lane_image);

    // Assert response
    let expected_stop_line = Line::new(Point::new(0.0, 0.0), Vector3::new(0.0, 0.0));
    let expected_obstacles = ObstacleList::from(vec![]);

    stop_line_topic.assert_message(expected_stop_line);
    obstacle_topic.assert_message(expected_obstacles);
    let messages = lane_topic.get_messages();
    let front_msg = messages
      .front()
      .expect("Message queue was empty after timeout")
      .to_owned();
    println!("{front_msg:?}");
    assert!(front_msg
      .centre
      .origin
      .point_eq(&Point::new(0.509012626389063, 0.7392593887128791)));
    assert!(front_msg
      .centre
      .direction
      .vec_eq(&Vector3::new(-0.005384461745997382, -0.22147785168950884)));
  }

  #[ros_test]
  fn test_detect_obstacles() {
    // Init topics
    let stop_line_topic = rostest::Topic::<Line>::create("/stop_line");
    let lane_topic = rostest::Topic::<Lane>::create("/lanes");
    let obstacle_topic = rostest::Topic::<ObstacleList>::create("/obstacles");

    // Create node
    rostest::instantiate_node(process_obstacle_image);

    // Assert response
    let expected_stop_line = Line::new(Point::new(0.0, 0.0), Vector3::new(0.0, 0.0));
    let expected_obstacles = ObstacleList::from(vec![
      Obstacle::new(
        Point::new(0.4405809879302979, 0.5192097028096517),
        6.7849393,
        Object {
          type_: Object::MIRTE,
        },
      ),
      Obstacle::new(
        Point::new(0.44025073051452634, 0.4123835563659668),
        20.952547,
        Object {
          type_: Object::DUCK,
        },
      ),
    ]);

    obstacle_topic.assert_message(expected_obstacles);
    stop_line_topic.assert_message(expected_stop_line);
    let messages = lane_topic.get_messages();
    let front_msg = messages
      .front()
      .expect("Message queue was empty after timeout")
      .to_owned();
    println!("{front_msg:?}");
    //    assert!(front_msg
    //      .centre
    //      .origin
    //      .point_eq(&Point::new(0.509012626389063, 0.7392593887128791)));
    //    assert!(front_msg
    //      .centre
    //      .direction
    //      .vec_eq(&Vector3::new(-0.005384461745997382, -0.22147785168950884)));
  }
}
