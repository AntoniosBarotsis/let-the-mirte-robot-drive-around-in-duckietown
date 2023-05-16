use thiserror::Error;

/// Represents any errors that can occur while performing ROS related tasks.
#[derive(Error, Debug)]
pub enum RosError {
  #[error("Expected a value between 0 and 100, got: {actual}.")]
  InvalidMotorValue { actual: i32 },
  #[error("Something went wrong while instantiating the \"{name}\" ROS client: {message}")]
  ClientCreation { name: String, message: String },
  #[error("{0}")]
  ClientResponse(String),
  #[error("{0}")]
  Generic(#[from] rosrust::error::tcpros::Error),
}
