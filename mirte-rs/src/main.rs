use cv::show_in_window;
use ros::{process_ros_image, CvImage};

/// For now, just reads an image from ROS and shows it on screen.
#[allow(clippy::unwrap_used)]
fn main() {
  process_ros_image(|img| {
    let mat = CvImage::from_imgmsg(img).unwrap().as_cvmat().unwrap();
    show_in_window(&mat);
  });
}
