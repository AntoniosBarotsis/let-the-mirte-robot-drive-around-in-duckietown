#![allow(elided_lifetimes_in_paths, unsafe_op_in_unsafe_fn)]

mod classes;

use classes::{PyColour, PyCvError, PyLane, PyLine, PyLineSegment, PyPoint, PyVector};
use cv::{
  detect_lines::detect_line_type,
  line::{Colour, LineSegment},
};
use mirte_rs::detect_lane::detect_lane;

#[cfg(feature = "dev")]
use cv::image::dbg_mat;
#[cfg(not(feature = "dev"))]
use mirte_rs::get_image;

use pyo3::prelude::*;

#[pyfunction(name = "detect_line_type")]
#[allow(clippy::unwrap_used, clippy::needless_pass_by_value)]
fn detect_line_type_py(colours: Vec<PyColour>) -> PyResult<Vec<PyLineSegment>> {
  // Gets image from Mirte's camera when "dev" is enabled, otherwise takes it from the assets
  // folder. This is done so it can be tested in CI as well as on the robot.
  #[cfg(not(feature = "dev"))]
  let mat = get_image().map_err(PyCvError::from)?;
  #[cfg(feature = "dev")]
  let mat = dbg_mat("./assets/input_1.jpg").map_err(PyCvError::from)?;

  let colours = colours.into_iter().map(Colour::from).collect::<Vec<_>>();

  let lines = detect_line_type(&mat, colours).map_err(PyCvError::from)?;

  let res = lines
    .into_iter()
    .map(PyLineSegment::from)
    .collect::<Vec<_>>();

  Ok(res)
}

// This is very much not unnecessary, it means the result can be Python's None
#[allow(clippy::unnecessary_wraps)]
#[pyfunction(name = "detect_lane")]
#[allow(clippy::unwrap_used, clippy::needless_pass_by_value)]
fn detect_lane_py(lines: Vec<PyLineSegment>) -> PyResult<PyLane> {
  let lines = lines.into_iter().map(LineSegment::from).collect::<Vec<_>>();

  let res = PyLane::from(detect_lane(&lines));

  Ok(res)
}

#[pymodule]
fn cv_py(_py: Python, m: &PyModule) -> PyResult<()> {
  m.add_class::<PyLane>()?;
  m.add_class::<PyLineSegment>()?;
  m.add_class::<PyLine>()?;
  m.add_class::<PyVector>()?;
  m.add_class::<PyColour>()?;
  m.add_class::<PyPoint>()?;

  m.add_function(wrap_pyfunction!(detect_line_type_py, m)?)?;
  m.add_function(wrap_pyfunction!(detect_lane_py, m)?)?;
  Ok(())
}
