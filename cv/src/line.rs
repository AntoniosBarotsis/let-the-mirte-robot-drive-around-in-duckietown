use derive_more::{Add, Div, Mul, Neg, Sub, Sum};

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_YELLOW: &[[u8; 3]; 2] = &[[20, 115, 115], [45, 255, 255]];
pub static HSV_WHITE: &[[u8; 3]; 2] = &[[0, 0, 165], [179, 55, 255]];

/// given a colour type it will return the lower and upper bound of the range of that colour in HSV
///
/// # Panics
///
/// * `colour` - The colour of which the colour range needs to be extracted
///
/// Return an 2d-array with the lower bound on index 0 and upper bound on index 1
///
/// # Examples
///
/// ```
/// use cv::line::{get_colour, Colour};
///
/// let yellow = *get_colour(Colour::Yellow);
/// let white = *get_colour(Colour::White);
/// assert_ne!(white, yellow);
/// ```
pub fn get_colour(colour: Colour) -> &'static [[u8; 3]; 2] {
  match colour {
    Colour::White => HSV_WHITE,
    Colour::Yellow => HSV_YELLOW,
    colour => panic!("No HSV constants defined for {colour:?}!"),
  }
}
// Represents a line
#[derive(Debug, Clone, Copy)]
pub struct Line {
  pub colour: Colour,
  pub start: Pos,
  pub end: Pos,
}

impl Line {
  pub fn new(colour: Colour, start: Pos, end: Pos) -> Self {
    Self { colour, start, end }
  }

  pub fn from_vector(vector: Vector, colour: Colour) -> Self {
    Self::new(
      colour,
      vector.origin,
      vector.origin + Pos::from(vector.dir),
    )
  }

  // TODO: Are we sure we need to make sure vectors point towards negative y? It seems unnecessary
  /// Transforms the line into a vector from (0, 0) pointing towards negative y
  pub fn direction(&self) -> Dir {
    if self.start.y < self.end.y {
      Dir::from_pos(self.start - self.end)
    } else {
      Dir::from_pos(self.end - self.start)
    }
  }

  pub fn midpoint(&self) -> Pos {
    (self.start + self.end) / 2.0
  }
}

// Represents a line color
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Colour {
  Red,
  Orange,
  Yellow,
  Green,
  Blue,
  Purple,
  Black,
  White,
}

// Represents a end coordinate of a line
#[derive(Debug, Clone, Copy, Add, Sub, Div, Mul, Sum)]
pub struct Pos {
  pub x: f32,
  pub y: f32,
}

impl Pos {
  pub fn new(x: f32, y: f32) -> Self {
    Self { x, y }
  }
}

impl From<Dir> for Pos {
    fn from(value: Dir) -> Self {
        Self { x: value.x, y: value.y }
    }
}

#[derive(Debug, Clone, Copy, Add, Sub, Div, Mul, Sum, Neg)]
pub struct Dir {
  pub x: f32,
  pub y: f32,
}

impl Dir {
  pub fn new(x: f32, y: f32) -> Self {
    Self { x, y }
  }

  fn from_pos(pos: Pos) -> Self {
    Self::new(pos.x, pos.y)
  }

  pub fn squared_length(&self) -> f32 {
    self.x * self.x + self.y * self.y
  }

  pub fn length(&self) -> f32 {
    f32::sqrt(self.squared_length())
  }
}

#[derive(Debug, Clone, Copy, Add, Sub, Div, Mul, Sum)]
pub struct Vector {
  pub origin: Pos,
  pub dir: Dir,
}

impl Vector {
  pub fn new(origin: Pos, dir: Dir) -> Self {
    Self { origin, dir }
  }

  pub fn from_line(line: &Line) -> Self {
    Self {
      origin: line.start,
      dir: line.direction(),
    }
  }

  pub fn midpoint(&self) -> Pos {
    self.origin + Pos::from(self.dir) / 2.0
  }

  pub fn end(&self) -> Pos {
    self.origin + Pos::from(self.dir)
  }

  pub fn length(&self) -> f32 {
    self.dir.length()
  }

  pub fn slope(&self) -> f32 {
    // If self.x is 0, the slope is invalid, and it means the line must be stictly vertical, which means simply
    // returning a high value suffices for the calculations we are doing.
    let slope = self.dir.y / self.dir.x;
    if slope.is_nan() {
      return 1000.0;
    }
    slope
  }

  pub fn intersect(&self, other: &Vector) -> Option<Pos> {
    let slope1 = self.slope();
    let slope2 = other.slope();

    let x = (self.origin.x * slope1 - self.origin.y - other.origin.x * slope2 + other.origin.y)
      / (slope1 - slope2);
    // If (slope1 - slope2) is 0, x will be invalid because the lines never intersect.
    if x.is_nan() {
      return None;
    }
    let y = slope1 * (x - self.origin.x) + self.origin.y;

    Some(Pos { x, y })
  }
}
