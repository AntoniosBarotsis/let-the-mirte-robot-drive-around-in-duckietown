#![allow(elided_lifetimes_in_paths, unsafe_op_in_unsafe_fn)]

mod classes;

use classes::{PyColour, PyCvError, PyLine, PyPos};
use cv::{
  cv_error::CvError,
  detect_lines::detect_line_type,
  line::{Colour, Line},
};
use mirte_rs::detect_lane::detect_lane;

// #[cfg(debug_assertions)]
use cv::image::dbg_mat;
// #[cfg(not(debug_assertions))]
// use mirte_rs::get_image;

use pyo3::prelude::*;

#[pyfunction(name = "detect_line_type")]
#[allow(clippy::unwrap_used, clippy::needless_pass_by_value)]
fn detect_line_type_py(colours: Vec<PyColour>) -> PyResult<Vec<PyLine>> {
  // Gets image from Mirte's camera when ran in release, otherwise takes it from the assets folder.
  // This is done so it can be tested in CI as well as on the robot.
  // #[cfg(not(debug_assertions))]
  // let mat = get_image();
  // #[cfg(debug_assertions)]
  let mat = dbg_mat("./assets/input_1.jpg").map_err(PyCvError::from)?;

  let colours = colours.into_iter().map(Colour::from).collect::<Vec<_>>();

  let lines = detect_line_type(&mat, colours).map_err(PyCvError::from)?;

  let res = lines.into_iter().map(PyLine::from).collect::<Vec<_>>();

  Ok(res)
}

// This is very much not unnecessary, it means the result can be Python's None
#[allow(clippy::unnecessary_wraps)]
#[pyfunction(name = "detect_lane")]
#[allow(clippy::unwrap_used, clippy::needless_pass_by_value)]
fn detect_lane_py(lines: Vec<PyLine>) -> PyResult<Vec<PyLine>> {
  let lines = lines.into_iter().map(Line::from).collect::<Vec<_>>();

  let res = detect_lane(&lines)
    .map(|res| res.into_iter().map(PyLine::from).collect::<Vec<_>>())
    .map_err(|_er| CvError::NoLinesDetected)
    .map_err(PyCvError::from)?;

  Ok(res)
}

#[pymodule]
fn cv_py(_py: Python, m: &PyModule) -> PyResult<()> {
  m.add_class::<PyLine>()?;
  m.add_class::<PyColour>()?;
  m.add_class::<PyPos>()?;

  m.add_function(wrap_pyfunction!(detect_line_type_py, m)?)?;
  m.add_function(wrap_pyfunction!(detect_lane_py, m)?)?;
  Ok(())
}
