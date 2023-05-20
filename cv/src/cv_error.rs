use thiserror::Error;

/// Represents any errors that can occur while performing Computer Vision related tasks.
#[derive(Error, Debug)]
pub enum CvError {
  #[error("An error occured instantiating the line detector")]
  LineDetectorCreation,
  #[error("No lines were detected in the image")]
  NoLinesDetected,
  #[error("Error drawing the lines on the image")]
  Drawing,
  #[error("Error during io: {0}")]
  Io(String),
  #[error("{0}")]
  Other(String),
  #[error("{0}")]
  ColorConversion(String),
  #[error("{0}")]
  OpenCvGeneric(#[from] opencv::Error),
}
