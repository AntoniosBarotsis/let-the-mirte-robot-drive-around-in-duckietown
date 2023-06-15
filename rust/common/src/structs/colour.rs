use crate::mirte_msgs::Colour;
use std::hash::{Hash, Hasher};

impl Colour {
  /// # Panics
  ///
  /// Will panic on an invalid color id (valid values are in [0,7]).
  pub fn parameter_name(&self) -> &'static str {
    match self.type_ {
      0 => "red",
      1 => "orange",
      2 => "yellow",
      3 => "green",
      4 => "blue",
      5 => "purple",
      6 => "black",
      7 => "white",
      _ => unreachable!("Invalid colour type {}", self.type_),
    }
  }
}

impl Copy for Colour {}
impl Eq for Colour {}

impl Hash for Colour {
  fn hash<H: Hasher>(&self, state: &mut H) {
    self.type_.hash(state);
  }
}
