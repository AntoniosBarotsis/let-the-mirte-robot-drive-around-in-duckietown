#![allow(elided_lifetimes_in_paths, unsafe_op_in_unsafe_fn)]

mod classes;

use classes::{PyColour, PyCvError, PyLine, PyPos};
use cv::{dbg_mat, detect_line_type, Colour};
use pyo3::prelude::*;

#[pyfunction(name = "detect_line_type")]
#[allow(clippy::unwrap_used, clippy::needless_pass_by_value)]
fn detect_line_type_py(colours: Vec<PyColour>) -> PyResult<Vec<PyLine>> {
  let mat = dbg_mat().map_err(PyCvError::from)?;
  let colours = colours.into_iter().map(Colour::from).collect::<Vec<_>>();

  let lines = detect_line_type(&mat, colours).map_err(PyCvError::from)?;

  let res = lines.into_iter().map(PyLine::from).collect::<Vec<_>>();

  Ok(res)
}

/// A Python module implemented in Rust.
#[pymodule]
fn cv_py(_py: Python, m: &PyModule) -> PyResult<()> {
  m.add_class::<PyLine>()?;
  m.add_class::<PyColour>()?;
  m.add_class::<PyPos>()?;
  //   m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
  m.add_function(wrap_pyfunction!(detect_line_type_py, m)?)?;
  Ok(())
}
