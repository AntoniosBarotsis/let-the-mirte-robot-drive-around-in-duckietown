use serde::{Deserialize, Serialize};

// H: 0-179, S: 0-255, V: 0-255
use crate::structs::colour::ColourEnum;

pub type ColourValue = [u8; 3];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Threshold {
  pub lower: ColourValue,
  pub upper: ColourValue,
}

impl Threshold {
  pub fn from(lower: ColourValue, upper: ColourValue) -> Self {
    Self { lower, upper }
  }

  /// given a colour type, it will return the default lower and upper bound of the range of that colour in HSV
  ///
  /// # Panics
  ///
  /// When called with a colour other than `White`, `Yellow` or `Red`.
  ///
  /// * `colour` - The colour of which the colour range needs to be extracted
  ///
  /// Return an 2d-array with the lower bound on index 0 and upper bound on index 1
  ///
  /// # Examples
  ///
  /// ```
  /// use common::structs::{colour::ColourEnum, threshold::Threshold};
  ///
  /// let yellow = Threshold::by_colour(ColourEnum::Yellow);
  /// let white = Threshold::by_colour(ColourEnum::White);
  /// assert_ne!(white, yellow);
  /// ```
  pub fn by_colour(colour: ColourEnum) -> Self {
    match colour {
      ColourEnum::White => Self {
        lower: [0, 0, 190],
        upper: [179, 40, 255],
      },
      ColourEnum::Yellow => Self {
        lower: [20, 100, 115],
        upper: [45, 255, 255],
      },
      ColourEnum::Red => Self {
        lower: [165, 90, 60],
        upper: [10, 255, 255],
      },
      _ => panic!("No colour threshold defined for {colour:?}!"),
    }
  }
}
