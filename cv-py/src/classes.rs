use cv::{
  cv_error::CvError,
  line::{Line, Vector},
};
use cv::{
  lane::Lane,
  line::{Colour, LineSegment, Point},
};
use mirte_rs::mirte_error::MirteError;
use pyo3::exceptions::{PyIOError, PyRuntimeError};
use pyo3::prelude::*;
use ros::RosError;

#[derive(Debug, Clone)]
#[pyclass(name = "Lane")]
pub(crate) struct PyLane {
  #[pyo3(get, set)]
  pub centre: PyLine,
  #[pyo3(get, set)]
  pub left: PyLine,
  #[pyo3(get, set)]
  pub right: PyLine,
}

#[pymethods]
impl PyLane {
  fn __str__(&self) -> String {
    self.to_string()
  }
}

impl ToString for PyLane {
  fn to_string(&self) -> String {
    format!(
      "Lane {{centre: {}, left: {}, right: {}}}",
      self.centre.to_string(),
      self.left.to_string(),
      self.right.to_string()
    )
  }
}

impl From<Lane> for PyLane {
  fn from(value: Lane) -> Self {
    let lane = PyLine::from(value.centre);
    let left_line = PyLine::from(value.left);
    let right_line = PyLine::from(value.right);

    PyLane {
      centre: lane,
      left: left_line,
      right: right_line,
    }
  }
}

#[derive(Debug, Clone)]
#[pyclass(name = "LineSegment")]
pub(crate) struct PyLineSegment {
  #[pyo3(get, set)]
  pub colour: PyColour,
  #[pyo3(get, set)]
  pub start: PyPoint,
  #[pyo3(get, set)]
  pub end: PyPoint,
}

#[pymethods]
impl PyLineSegment {
  fn __str__(&self) -> String {
    self.to_string()
  }
}

impl ToString for PyLineSegment {
  fn to_string(&self) -> String {
    format!(
      "LineSegment {{colour: {}, start: {}, end: {}}}",
      self.colour.to_string(),
      self.start.to_string(),
      self.end.to_string()
    )
  }
}

impl From<LineSegment> for PyLineSegment {
  fn from(value: LineSegment) -> Self {
    let colour = PyColour::from(value.colour);
    let start = PyPoint::from(value.start);
    let end = PyPoint::from(value.end);

    PyLineSegment { colour, start, end }
  }
}

impl From<PyLineSegment> for LineSegment {
  fn from(value: PyLineSegment) -> Self {
    let colour = Colour::from(value.colour);
    let start = Point::from(value.start);
    let end = Point::from(value.end);

    LineSegment { colour, start, end }
  }
}

#[derive(Debug, Clone)]
#[pyclass(name = "Line")]
pub(crate) struct PyLine {
  #[pyo3(get, set)]
  pub origin: PyPoint,
  #[pyo3(get, set)]
  pub direction: PyVector,
}

#[pymethods]
impl PyLine {
  fn __str__(&self) -> String {
    self.to_string()
  }
}

impl ToString for PyLine {
  fn to_string(&self) -> String {
    format!(
      "Line {{origin: {}, direction: {}}}",
      self.origin.to_string(),
      self.direction.to_string()
    )
  }
}

impl From<Line> for PyLine {
  fn from(value: Line) -> Self {
    let origin = PyPoint::from(value.origin);
    let direction = PyVector::from(value.dir);

    Self { origin, direction }
  }
}

#[derive(Debug, Clone)]
#[pyclass(name = "Vector")]
pub(crate) struct PyVector {
  #[pyo3(get, set)]
  pub x: f32,
  #[pyo3(get, set)]
  pub y: f32,
}

#[pymethods]
impl PyVector {
  fn __str__(&self) -> String {
    self.to_string()
  }
}

impl ToString for PyVector {
  fn to_string(&self) -> String {
    format!("Vector: {{x: {}, y: {}}}", self.x, self.y)
  }
}

impl From<Vector> for PyVector {
  fn from(value: Vector) -> Self {
    let x = value.x;
    let y = value.y;

    Self { x, y }
  }
}

#[derive(Debug, Clone, Copy)]
#[pyclass(name = "Colour")]
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

#[pymethods]
impl PyColour {
  #[allow(clippy::trivially_copy_pass_by_ref)]
  fn __str__(&self) -> String {
    self.to_string()
  }
}

impl ToString for PyColour {
  fn to_string(&self) -> String {
    match self {
      PyColour::Red => "Colour: red".to_string(),
      PyColour::Orange => "Colour: orange".to_string(),
      PyColour::Yellow => "Colour: yellow".to_string(),
      PyColour::Green => "Colour: green".to_string(),
      PyColour::Blue => "Colour: blue".to_string(),
      PyColour::Purple => "Colour: purple".to_string(),
      PyColour::Black => "Colour: black".to_string(),
      PyColour::White => "Colour: white".to_string(),
    }
  }
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
#[pyclass(name = "Point")]
pub(crate) struct PyPoint {
  #[pyo3(get, set)]
  pub x: f32,
  #[pyo3(get, set)]
  pub y: f32,
}

#[pymethods]
impl PyPoint {
  fn __str__(&self) -> String {
    self.to_string()
  }
}

impl ToString for PyPoint {
  fn to_string(&self) -> String {
    format!("Point: {{x: {}, y: {}}}", self.x, self.y)
  }
}

impl From<Point> for PyPoint {
  fn from(value: Point) -> Self {
    let x = value.x;
    let y = value.y;

    PyPoint { x, y }
  }
}

impl From<PyPoint> for Point {
  fn from(value: PyPoint) -> Self {
    let x = value.x;
    let y = value.y;

    Point { x, y }
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
