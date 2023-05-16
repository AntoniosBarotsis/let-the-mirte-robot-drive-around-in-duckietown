/// Utility for driving Mirte around.
pub mod drive;
pub mod ros_error;

use cv_bridge::msgs::sensor_msgs::Image;

/// Intermediate stage between a `ROS` and an `OpenCV` image.
pub use cv_bridge::CvImage;

/// This reads an image from the ROS `/webcam/image_raw` topic and runs the callback on it.
///
/// Note that it blocks the current thread and shifts execution to inside the callback.
///
/// # Example
///
/// ```no_run
/// use ros::process_ros_image;
///
/// process_ros_image(|img| {
///   // Convert to opencv::mat
///   let mat = cv_bridge::CvImage::from_imgmsg(img).unwrap().as_cvmat().unwrap();
///
///   // Do other stuff here...
/// });
/// ```
pub fn process_ros_image<T>(callback: T)
where
  T: Fn(Image) + Send + 'static,
{
  env_logger::init();

  // Initialize node
  rosrust::init("webcam_listener");

  // Create subscriber
  // The subscriber is stopped when the returned object is destroyed
  let _subscriber_raii = rosrust::subscribe("/webcam/image_raw", 1, move |img: Image| {
    rosrust::ros_info!("Image received.");

    callback(img);
  })
  .expect("Create subscriber");

  // Block the thread until a shutdown signal is received
  rosrust::spin();
}
