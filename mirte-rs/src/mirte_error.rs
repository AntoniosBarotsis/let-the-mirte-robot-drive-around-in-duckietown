use thiserror::Error;

/// Represents any errors that can occur while performing ROS related tasks.
#[derive(Error, Debug)]
pub enum MirteError {
  #[error("{0}")]
  Ros(#[from] ros::RosError),
  #[error("{0}")]
  Cv(#[from] cv::cv_error::CvError),
}
