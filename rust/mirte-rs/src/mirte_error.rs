use thiserror::Error;

#[derive(Error, Debug)]
pub enum MirteError {
  #[error("{0}")]
  Ros(#[from] ros::RosError),
  #[error("{0}")]
  Cv(#[from] cv::cv_error::CvError),
}
