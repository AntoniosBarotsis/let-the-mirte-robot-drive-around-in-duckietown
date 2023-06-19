use crate::mirte_msgs::Colour;
use std::hash::{Hash, Hasher};

/// A slightly more convenient wrapper around [`Colour`].
///
/// # Example
///
/// ```
/// # use common::{mirte_msgs::Colour, structs::colour::ColourEnum};
/// // Instead of
/// let c_1 = Colour { type_: Colour::RED };
///
/// // You can do
/// let c_2 = ColourEnum::Red.into();
///
/// assert_eq!(c_1, c_2);
/// ```
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum ColourEnum {
  Red,
  Orange,
  Yellow,
  Green,
  Blue,
  Purple,
  Black,
  White,
}

impl ColourEnum {
  pub fn parameter_name(&self) -> &'static str {
    match self {
      ColourEnum::Red => "red",
      ColourEnum::Orange => "orange",
      ColourEnum::Yellow => "yellow",
      ColourEnum::Green => "green",
      ColourEnum::Blue => "blue",
      ColourEnum::Purple => "purple",
      ColourEnum::Black => "black",
      ColourEnum::White => "white",
    }
  }
}

impl From<ColourEnum> for Colour {
  fn from(value: ColourEnum) -> Self {
    match value {
      ColourEnum::Red => Colour { type_: Colour::RED },
      ColourEnum::Orange => Colour {
        type_: Colour::ORANGE,
      },
      ColourEnum::Yellow => Colour {
        type_: Colour::YELLOW,
      },
      ColourEnum::Green => Colour {
        type_: Colour::GREEN,
      },
      ColourEnum::Blue => Colour {
        type_: Colour::BLUE,
      },
      ColourEnum::Purple => Colour {
        type_: Colour::PURPLE,
      },
      ColourEnum::Black => Colour {
        type_: Colour::BLACK,
      },
      ColourEnum::White => Colour {
        type_: Colour::WHITE,
      },
    }
  }
}

impl From<Colour> for ColourEnum {
  fn from(value: Colour) -> Self {
    match value.type_ {
      0 => ColourEnum::Red,
      1 => ColourEnum::Orange,
      2 => ColourEnum::Yellow,
      3 => ColourEnum::Green,
      4 => ColourEnum::Blue,
      5 => ColourEnum::Purple,
      6 => ColourEnum::Black,
      7 => ColourEnum::White,
      _ => unreachable!("Invalid colour type {}", value.type_),
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
