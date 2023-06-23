use core::process_mat;
use ros::{process_ros_image, CvImage};
use std::error::Error;

/// For now, just reads an image from ROS and shows it on screen.
#[allow(clippy::unwrap_used, clippy::expect_used)]
fn main() -> Result<(), Box<dyn Error>> {
  let thresholds = ros::param::get_thresholds();

  process_ros_image(move |img| {
    let mat = CvImage::from_imgmsg(img).unwrap().as_cvmat().unwrap();

    // This clone here, although seemingly useless, fixes a weird bug that causes artifacts to
    // appear during the color conversion. For more details, refer to:
    // https://github.com/twistedfall/opencv-rust/issues/277
    #[allow(clippy::redundant_clone)]
    let mat = mat.clone();

    process_mat(&mat, &thresholds);
  })?;

  Ok(())
}
