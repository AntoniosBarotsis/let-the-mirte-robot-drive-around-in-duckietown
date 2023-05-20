use cv::show_in_window;
use ros::{process_ros_image, CvImage};

/// For now, just reads an image from ROS and shows it on screen.
#[allow(clippy::unwrap_used)]
fn main() {
  let res = process_ros_image(|img| {
    let mat = CvImage::from_imgmsg(img).unwrap().as_cvmat().unwrap();

    // This clone here, although seemingly useless, fixes a weird bug that causes artifacts to
    // appear during the color conversion. For more details, refer to:
    // https://github.com/twistedfall/opencv-rust/issues/277
    #[allow(clippy::redundant_clone)]
    let mat = mat.clone();

    match show_in_window(&mat) {
      Ok(_) => {}
      Err(e) => panic!("Could not open image window: {e}"),
    }
  });

  if let Err(e) = res {
    eprintln!("{e}");
  }
}
