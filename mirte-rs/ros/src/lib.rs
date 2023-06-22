use std::{
  sync::{Arc, Condvar, Mutex},
  time::Duration,
};

pub mod drive;
pub mod param;
pub mod publishers;
pub mod ros_error;
#[cfg(test)]
pub mod test;

use cv_bridge::msgs::sensor_msgs::Image;

pub use common::mirte_msgs;
/// Intermediate stage between a `ROS` and an `OpenCV` image.
pub use cv_bridge::CvImage;
pub use ros_error::RosError;

use std::sync::Once;

static INIT: Once = Once::new();

/// Initializes the logger and the ROS node exactly once.
pub(crate) fn init() {
  INIT.call_once(|| {
    env_logger::init();

    // Initialize node
    rosrust::init("duckietown_navigator");
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
/// use cv_bridge::CvImage;
///
///
/// process_ros_image(|img| {
///   // Convert to opencv::mat
///   let mat = CvImage::from_imgmsg(img).unwrap().as_cvmat().unwrap();
///
///   // Do other stuff here...
/// });
/// ```
pub fn process_ros_image<T>(callback: T) -> Result<(), RosError>
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
  .map_err(|e| RosError::SubscriberCreation(e.to_string()))?;

  // Block the thread until a shutdown signal is received
  rosrust::spin();

  Ok(())
}

/// Retrieves a single image from the ROS `/webcam/image_raw` topic in a blocking manner. It will
/// block for a maximum of 5 seconds before returning an error.
///
/// # Panics
///
/// This function expects [`rosrust::subscribe`] to work on the `/webcam/image_raw` topic. If that
/// is not the case, it will panic as that is an unrecoverable error.
pub fn process_ros_image_one() -> Result<Image, RosError> {
  let arcmut: Arc<(Mutex<Image>, Condvar)> =
    Arc::new((Mutex::new(Image::default()), Condvar::new()));
  let arcmut_clone: Arc<(Mutex<Image>, Condvar)> = arcmut.clone();

  init();

  // Create subscriber
  // The subscriber is stopped when the returned object is destroyed
  #[allow(clippy::expect_used)]
  let _subscriber_raii = rosrust::subscribe("/webcam/image_raw", 1, move |img: Image| {
    // let img = callback(img);

    let (ref img_mutex, ref cond) = *Arc::clone(&arcmut_clone);

    // Here we are unwrapping the potential poison error which, since we are using one thread,
    // should never occur. If it did it would be unrecoverable so crashing here is fine.
    #[allow(clippy::unwrap_used)]
    {
      *img_mutex.lock().unwrap() = img;
    }

    cond.notify_all();
  })
  .expect("Create subscriber");

  #[allow(clippy::unwrap_used)]
  let guard = arcmut
    .1
    .wait_timeout(arcmut.0.lock().unwrap(), Duration::from_secs(5))
    .unwrap();

  // Check for timeout
  if guard.1.timed_out() {
    return Err(RosError::Timeout("No image received.".to_owned()));
  }

  let res_img = guard.0.clone();

  Ok(res_img)
}
