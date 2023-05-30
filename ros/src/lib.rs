use std::{
  sync::{Arc, Condvar, Mutex},
  time::Duration,
};

use cv_bridge::msgs::sensor_msgs::Image;
pub use cv_bridge::CvImage;

use std::sync::Once;

static INIT: Once = Once::new();

/// Initializes the logger and the webcam listener node exactly once.
fn init() {
  INIT.call_once(|| {
    env_logger::init();

    // Initialize node
    rosrust::init("webcam_listener");
  });
}

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
  init();

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

pub fn process_ros_image_one() -> Result<Image, ()> {
  let arcmut: Arc<(Mutex<Image>, Condvar)> =
    Arc::new((Mutex::new(Image::default()), Condvar::new()));
  let arcmut_clone: Arc<(Mutex<Image>, Condvar)> = arcmut.clone();

  init();

  // Create subscriber
  // The subscriber is stopped when the returned object is destroyed
  let _subscriber_raii = rosrust::subscribe("/webcam/image_raw", 1, move |img: Image| {
    rosrust::ros_info!("Image received.");

    // let img = callback(img);

    let (ref img_mutex, ref cond) = *Arc::clone(&arcmut_clone);
    *img_mutex.lock().unwrap() = img;
    cond.notify_all();
  })
  .expect("Create subscriber");

  let guard = arcmut
    .1
    .wait_timeout(arcmut.0.lock().unwrap(), Duration::from_secs(5))
    .unwrap();
  let res_img = guard.0.clone();

  Ok(res_img)
}
