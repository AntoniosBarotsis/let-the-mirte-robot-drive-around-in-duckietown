use cv::image::convert_to_rgb;
use mirte_rs::process_mat;
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
    let mat = convert_to_rgb(&mat.clone()).expect("bla");

    process_mat(mat);
  });

  if let Err(e) = res {
    eprintln!("{e}");
  }
}
