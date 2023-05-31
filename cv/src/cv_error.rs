use thiserror::Error;

/// Represents any errors that can occur while performing Computer Vision related tasks.
#[derive(Error, Debug)]
pub enum CvError {
  #[error("An error occured when converting colours")]
  ColourConversion,
  #[error("An error occured instantiating the line detector")]
  LineDetectorCreation,
  #[error("No lines were detected in the image")]
  NoLinesDetected,
  #[error("Error drawing the lines on the image")]
  Drawing,
  #[error("Error during io: {0}")]
  IoError(String),
  #[error("{0}")]
  Other(String),
}

impl From<opencv::Error> for CvError {
  fn from(value: opencv::Error) -> Self {
    CvError::Other(value.message)
  }
}
