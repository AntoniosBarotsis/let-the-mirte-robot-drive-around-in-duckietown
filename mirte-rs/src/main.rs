use cv::{convert_to_rgb, draw_lines::draw_lines};
use mirte_rs::detect_lane::detect_lane;
use ros::{process_ros_image, CvImage};

/// For now, just reads an image from ROS and shows it on screen.
#[allow(clippy::unwrap_used)]
fn main() {
  process_ros_image(|img| {
    let mat = CvImage::from_imgmsg(img).unwrap().as_cvmat().unwrap();

    // This clone here, although seemingly useless, fixes a weird bug that causes artifacts to
    // appear during the color conversion. For more details, refer to:
    // https://github.com/twistedfall/opencv-rust/issues/277
    #[allow(clippy::redundant_clone)]
    let mut mat = convert_to_rgb(&mat.clone()).expect("bla");

    let colours = vec![cv::line::Colour::Yellow, cv::line::Colour::White];
    let lines = cv::detect_line_type(&mat, colours).expect("Unable to detect line with cv");

    let lane = detect_lane(&lines).expect("Unable to detect the lane");

    let drawn_lines = [lines, lane].concat();
    //lines.push(lane);
    draw_lines(&mut mat, &drawn_lines);

    // show_in_window(&mat);
  });
}
