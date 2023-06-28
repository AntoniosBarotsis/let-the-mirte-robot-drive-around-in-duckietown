use thiserror::Error;

/// Represents any errors that can occur while performing ROS related tasks.
#[derive(Error, Debug)]
pub enum RosError {
  #[error("Something went wrong while instantiating the \"{name}\" ROS client: {message}")]
  ClientCreation { name: String, message: String },
  #[error("{0}")]
  ClientResponse(String),
  #[error("{0}")]
  SubscriberCreation(String),
  #[error("A timeout occured: {0}")]
  Timeout(String),
  #[error("{0}")]
  Generic(#[from] rosrust::error::tcpros::Error),
  #[error("Error instantiating publisher \"{topic}\"")]
  Publisher { topic: String },
}
