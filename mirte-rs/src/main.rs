use std::time::Instant;

use cv::downscale;
use mirte_rs::detect_lane::detect_lane;
use ros::{process_ros_image, CvImage};

/// For now, just reads an image from ROS and shows it on screen.
#[allow(clippy::unwrap_used)]
fn main() {
  process_ros_image(|img| {
    let time_total = Instant::now();

    let mat = CvImage::from_imgmsg(img).unwrap().as_cvmat().unwrap();

    // This clone here, although seemingly useless, fixes a weird bug that causes artifacts to
    // appear during the color conversion. For more details, refer to:
    // https://github.com/twistedfall/opencv-rust/issues/277
    #[allow(clippy::redundant_clone)]
    // let mat = convert_to_rgb(&mat.clone()).expect("bla");
    let time_1 = Instant::now();
    let resized = downscale(&mat).expect("downscale");
    println!("resizing: {:?}", time_1.elapsed());

    let colours = vec![cv::line::Colour::Yellow, cv::line::Colour::White];

    let time_2 = Instant::now();
    let lines = cv::detect_line_type(&resized, colours).expect("Unable to detect line with cv");
    println!("detecting lines: {:?}", time_2.elapsed());

    let time_3 = Instant::now();
    let _drawn_lines = if let Ok(lane) = detect_lane(&lines) {
      [lines, lane].concat()
    } else {
      lines
    };
    println!("detecting lane: {:?}", time_3.elapsed());
    //lines.push(lane);
    // draw_lines(&mut resized, &drawn_lines);

    // show_in_window(&mat);

    println!("total: {:?}", time_total.elapsed());
  });
}
