use derive_more::{Add, Div, Mul, Neg, Sub, Sum};

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_YELLOW: &[[u8; 3]; 2] = &[[25, 50, 50], [45, 255, 255]];
pub static HSV_WHITE: &[[u8; 3]; 2] = &[[0, 0, 150], [179, 60, 255]];
pub static HSV_GREEN: &[[u8; 3]; 2] = &[[45, 0, 0], [90, 255, 255]];

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
      vector.origin + Pos::from_dir(vector.dir),
    )
  }

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

  pub fn from_dir(dir: Dir) -> Self {
    Self::new(dir.x, dir.y)
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
    self.origin + Pos::from_dir(self.dir) / 2.0
  }

  pub fn end(&self) -> Pos {
    self.origin + Pos::from_dir(self.dir)
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

#[cfg(test)]
mod tests {
  use crate::line::{Colour, Dir, Line, Pos, Vector};
  use float_cmp::approx_eq;

  #[test]
  fn create_pos() {
    let pos = Pos::new(10.0, -5.0);
    assert!(approx_eq!(f32, pos.x, 10.0, ulps = 2));
    assert!(approx_eq!(f32, pos.y, -5.0, ulps = 2));
  }

  #[test]
  fn create_pos_from_dir() {
    let dir = Dir::new(10.0, -5.0);
    let pos = Pos::from_dir(dir);
    assert!(approx_eq!(f32, pos.x, 10.0, ulps = 2));
    assert!(approx_eq!(f32, pos.y, -5.0, ulps = 2));
  }

  #[test]
  fn create_dir() {
    let dir = Dir::new(10.0, -5.0);
    assert!(approx_eq!(f32, dir.x, 10.0, ulps = 2));
    assert!(approx_eq!(f32, dir.y, -5.0, ulps = 2));
  }

  #[test]
  fn create_dir_from_pos() {
    let pos = Pos::new(10.0, -5.0);
    let dir = Dir::from_pos(pos);
    assert!(approx_eq!(f32, dir.x, 10.0, ulps = 2));
    assert!(approx_eq!(f32, dir.y, -5.0, ulps = 2));
  }

  #[test]
  fn compute_squared_length() {
    let dir = Dir::new(10.0, -5.0);
    assert!(approx_eq!(f32, dir.squared_length(), 125.0, ulps = 2));
  }

  #[test]
  fn compute_length() {
    let dir = Dir::new(3.0, -4.0);
    assert!(approx_eq!(f32, dir.length(), 5.0, ulps = 2));
  }

  #[test]
  fn create_line() {
    let line = Line::new(Colour::Red, Pos::new(10.0, -5.0), Pos::new(20.0, -10.0));
    assert_eq!(line.colour, Colour::Red);
    assert!(approx_eq!(f32, line.start.x, 10.0, ulps = 2));
    assert!(approx_eq!(f32, line.start.y, -5.0, ulps = 2));
    assert!(approx_eq!(f32, line.end.x, 20.0, ulps = 2));
    assert!(approx_eq!(f32, line.end.y, -10.0, ulps = 2));
  }

  #[test]
  fn create_line_from_vector() {
    let vector = Vector::new(Pos::new(10.0, -5.0), Dir::new(10.0, -5.0));
    let line = Line::from_vector(vector, Colour::Red);
    assert_eq!(line.colour, Colour::Red);
    assert!(approx_eq!(f32, line.start.x, 10.0, ulps = 2));
    assert!(approx_eq!(f32, line.start.y, -5.0, ulps = 2));
    assert!(approx_eq!(f32, line.end.x, 20.0, ulps = 2));
    assert!(approx_eq!(f32, line.end.y, -10.0, ulps = 2));
  }

  #[test]
  fn get_direction() {
    let line = Line::new(Colour::Red, Pos::new(10.0, -5.0), Pos::new(20.0, -10.0));
    let dir = line.direction();
    assert!(approx_eq!(f32, dir.x, 10.0, ulps = 2));
    assert!(approx_eq!(f32, dir.y, -5.0, ulps = 2));
  }

  #[test]
  fn get_direction_from_inverted_coords() {
    let line = Line::new(Colour::Red, Pos::new(10.0, 5.0), Pos::new(20.0, 10.0));
    let dir = line.direction();
    assert!(approx_eq!(f32, dir.x, -10.0, ulps = 2));
    assert!(approx_eq!(f32, dir.y, -5.0, ulps = 2));
  }

  #[test]
  fn get_midpoint() {
    let line = Line::new(Colour::Red, Pos::new(10.0, -5.0), Pos::new(20.0, -10.0));
    let midpoint = line.midpoint();
    assert!(approx_eq!(f32, midpoint.x, 15.0, ulps = 2));
    assert!(approx_eq!(f32, midpoint.y, -7.5, ulps = 2));
  }
}
