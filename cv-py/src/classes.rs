use cv::cv_error::CvError;
use cv::line::{Colour, Line, LineSegment, Point, Pos};
use mirte_rs::mirte_error::MirteError;
use pyo3::exceptions::{PyIOError, PyRuntimeError};
use pyo3::prelude::*;
use ros::RosError;

#[derive(Debug, Clone)]
#[pyclass(name = "line")]
pub(crate) struct PyLine {
  #[pyo3(get, set)]
  pub colour: PyColour,
  #[pyo3(get, set)]
  pub start: PyPos,
  #[pyo3(get, set)]
  pub end: PyPos,
}

impl From<LineSegment> for PyLine {
  fn from(value: LineSegment) -> Self {
    let colour = PyColour::from(value.colour);
    let start = PyPos::from(value.start);
    let end = PyPos::from(value.end);

    PyLine { colour, start, end }
  }
}

impl From<PyLine> for Line {
  fn from(value: PyLine) -> Self {
    let colour = Colour::from(value.colour);
    let start = Pos::from(value.start);
    let end = Pos::from(value.end);

    Line { colour, start, end }
  }
}

#[derive(Debug, Clone, Copy)]
#[pyclass(name = "colour")]
#[allow(nonstandard_style)]
pub(crate) enum PyColour {
  #[pyo3(name = "red")]
  Red,
  #[pyo3(name = "orange")]
  Orange,
  #[pyo3(name = "yellow")]
  Yellow,
  #[pyo3(name = "green")]
  Green,
  #[pyo3(name = "blue")]
  Blue,
  #[pyo3(name = "purple")]
  Purple,
  #[pyo3(name = "black")]
  Black,
  #[pyo3(name = "white")]
  White,
}

impl From<Colour> for PyColour {
  fn from(value: Colour) -> Self {
    match value {
      Colour::Red => PyColour::Red,
      Colour::Orange => PyColour::Orange,
      Colour::Yellow => PyColour::Yellow,
      Colour::Green => PyColour::Green,
      Colour::Blue => PyColour::Blue,
      Colour::Purple => PyColour::Purple,
      Colour::Black => PyColour::Black,
      Colour::White => PyColour::White,
    }
  }
}

impl From<PyColour> for Colour {
  fn from(value: PyColour) -> Self {
    match value {
      PyColour::Red => Colour::Red,
      PyColour::Orange => Colour::Orange,
      PyColour::Yellow => Colour::Yellow,
      PyColour::Green => Colour::Green,
      PyColour::Blue => Colour::Blue,
      PyColour::Purple => Colour::Purple,
      PyColour::Black => Colour::Black,
      PyColour::White => Colour::White,
    }
  }
}

#[derive(Debug, Clone)]
#[pyclass(name = "pos")]
pub(crate) struct PyPos {
  #[pyo3(get, set)]
  pub x: f32,
  #[pyo3(get, set)]
  pub y: f32,
}

impl From<Point> for PyPos {
  fn from(value: Point) -> Self {
    let x = value.x;
    let y = value.y;

    PyPos { x, y }
  }
}

impl From<PyPos> for Pos {
  fn from(value: PyPos) -> Self {
    let x = value.x;
    let y = value.y;

    Pos { x, y }
  }
}

#[derive(Debug)]
pub(crate) struct PyCvError {
  pub(crate) msg: String,
  pub(crate) err: Box<MirteError>,
}

impl From<MirteError> for PyCvError {
  fn from(value: MirteError) -> Self {
    match value {
      MirteError::Ros(e) => Self::from(e),
      MirteError::Cv(e) => Self::from(e),
    }
  }
}

impl From<RosError> for PyCvError {
  fn from(value: RosError) -> Self {
    let msg = value.to_string();

    PyCvError {
      msg,
      err: Box::new(MirteError::Ros(value)),
    }
  }
}

impl From<CvError> for PyCvError {
  fn from(value: CvError) -> Self {
    let msg = value.to_string();

    PyCvError {
      msg,
      err: Box::new(MirteError::Cv(value)),
    }
  }
}

impl From<PyCvError> for PyErr {
  fn from(value: PyCvError) -> Self {
    let msg = value.msg;

    match value.err.as_ref() {
      MirteError::Ros(_) => unreachable!(),
      MirteError::Cv(err) => match err {
        CvError::NoLinesDetected
        | CvError::LineDetectorCreation
        | CvError::Other(_)
        | CvError::ColourConversion => PyErr::new::<PyRuntimeError, _>(msg),
        CvError::IoError(_) => PyErr::new::<PyIOError, _>(msg),
        CvError::Drawing => unreachable!(),
      },
    }
  }
}
