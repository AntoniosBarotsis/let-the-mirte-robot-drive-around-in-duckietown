use derive_more::{Add, Div, Mul, Neg, Sub, Sum};
use float_cmp::approx_eq;

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_YELLOW: &[[u8; 3]; 2] = &[[20, 115, 115], [45, 255, 255]];
pub static HSV_WHITE: &[[u8; 3]; 2] = &[[0, 0, 190], [179, 40, 255]];

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
    self.dir.y / self.dir.x
  }

  pub fn clamped_slope(&self) -> f32 {
    let slope = self.slope();
    if slope.is_nan() || slope.is_infinite() || slope > 100_000_000_000.0 {
      100_000_000_000.0
    } else {
      slope
    }
  }

  /// Returns the y value at the given x value on the vector
  fn y(&self, x: f32) -> f32 {
    self.slope() * (x - self.origin.x) + self.origin.y
  }

  pub fn intersect(&self, other: &Vector) -> Option<Pos> {
    // We clamp the slopes to still get a valid output even if the slopes are infinite or NaN
    let slope1 = self.clamped_slope();
    let slope2 = other.clamped_slope();
    // If slopes are the same, x will be invalid because the lines never intersect.
    if approx_eq!(f32, slope1, slope2, ulps = 2) {
      return None;
    }

    let x = (self.origin.x * slope1 - self.origin.y - other.origin.x * slope2 + other.origin.y)
      / (slope1 - slope2);
    let y = if self.slope().is_infinite() {
      other.y(x)
    } else {
      self.y(x)
    };

    Some(Pos { x, y })
  }
}

#[cfg(test)]
mod tests {
  use crate::line::{Colour, Dir, Line, Pos, Vector};
  use float_cmp::assert_approx_eq;

  fn assert_float_eq(f1: f32, f2: f32) {
    assert_approx_eq!(f32, f1, f2, ulps = 0);
  }

  fn assert_pos_eq(pos1: Pos, pos2: Pos) {
    assert_float_eq(pos1.x, pos2.x);
    assert_float_eq(pos1.y, pos2.y);
  }

  fn assert_dir_eq(dir1: Dir, dir2: Dir) {
    assert_float_eq(dir1.x, dir2.x);
    assert_float_eq(dir1.y, dir2.y);
  }

  fn assert_line_eq(line1: Line, line2: Line) {
    assert_eq!(line1.colour, line2.colour);
    assert_pos_eq(line1.start, line2.start);
    assert_pos_eq(line1.end, line2.end);
  }

  #[test]
  fn create_pos() {
    let pos = Pos::new(10.0, -5.0);
    assert_float_eq(pos.x, 10.0);
    assert_float_eq(pos.y, -5.0);
  }

  #[test]
  fn create_vector() {
    let vec = Vector::new(Pos::new(6.5, 3.0), Dir::new(1.0, 1.0));
    assert_pos_eq(vec.origin, Pos::new(6.5, 3.0));
    assert_dir_eq(vec.dir, Dir::new(1.0, 1.0));
  }

  #[test]
  fn vector_midpoint() {
    let vec = Vector::new(Pos::new(5.0, 0.0), Dir::new(0.0, 10.0));
    let midpoint = vec.midpoint();
    assert_pos_eq(midpoint, Pos::new(5.0, 5.0));
  }

  #[test]
  fn vector_end() {
    let vec = Vector::new(Pos::new(5.0, 0.0), Dir::new(0.0, 10.0));
    let end = vec.end();
    assert_pos_eq(end, Pos::new(5.0, 10.0));
  }

  #[test]
  fn vector_inf_slope() {
    let vec = Vector::new(Pos::new(5.0, 0.0), Dir::new(0.0, 10.0));
    let slope = vec.slope();
    assert_float_eq(slope, f32::INFINITY);
  }

  #[test]
  fn vector_slope() {
    let vec = Vector::new(Pos::new(5.0, 0.0), Dir::new(1.0, 10.0));
    let slope = vec.slope();
    assert_float_eq(slope, 10.0);
  }

  #[test]
  fn intersect() {
    let vec1 = Vector::new(Pos::new(4.0, 0.0), Dir::new(1.0, 1.0));
    let vec2 = Vector::new(Pos::new(0.0, 0.0), Dir::new(1.0, -1.0));
    let intersection = vec1.intersect(&vec2).expect("No intersection found!");
    assert_pos_eq(intersection, Pos::new(2.0, -2.0));
  }

  #[test]
  fn intersect_with_vertical() {
    let vec1 = Vector::new(Pos::new(4.0, 0.0), Dir::new(0.0, 1.0));
    let vec2 = Vector::new(Pos::new(0.0, 0.0), Dir::new(1.0, 1.0));
    let intersection = vec1.intersect(&vec2).expect("No intersection found!");
    assert_pos_eq(intersection, Pos::new(4.0, 4.0));
  }

  #[test]
  fn intersect_with_identical() {
    let vec1 = Vector::new(Pos::new(4.0, 0.0), Dir::new(1.0, 1.0));
    let vec2 = Vector::new(Pos::new(4.0, 0.0), Dir::new(1.0, 1.0));
    let intersection = vec1.intersect(&vec2);
    assert!(intersection.is_none());
  }

  #[test]
  fn intersect_with_same_slope() {
    let vec1 = Vector::new(Pos::new(4.0, 0.0), Dir::new(1.0, 1.0));
    let vec2 = Vector::new(Pos::new(8.0, 0.0), Dir::new(1.0, 1.0));
    let intersection = vec1.intersect(&vec2);
    assert!(intersection.is_none());
  }

  #[test]
  fn create_pos_from_dir() {
    let dir = Dir::new(10.0, -5.0);
    let pos = Pos::from_dir(dir);
    assert_pos_eq(pos, Pos::new(10.0, -5.0));
  }

  #[test]
  fn create_dir() {
    let dir = Dir::new(10.0, -5.0);
    assert_dir_eq(dir, Dir::new(10.0, -5.0));
  }

  #[test]
  fn create_dir_from_pos() {
    let pos = Pos::new(10.0, -5.0);
    let dir = Dir::from_pos(pos);
    assert_dir_eq(dir, Dir::new(10.0, -5.0));
  }

  #[test]
  fn compute_squared_length() {
    let dir = Dir::new(10.0, -5.0);
    assert_float_eq(dir.squared_length(), 125.0);
  }

  #[test]
  fn compute_length() {
    let dir = Dir::new(3.0, -4.0);
    assert_float_eq(dir.length(), 5.0);
  }

  #[test]
  fn create_line() {
    let line = Line::new(Colour::Red, Pos::new(10.0, -5.0), Pos::new(20.0, -10.0));
    assert_line_eq(
      line,
      Line::new(Colour::Red, Pos::new(10.0, -5.0), Pos::new(20.0, -10.0)),
    );
  }

  #[test]
  fn create_line_from_vector() {
    let vector = Vector::new(Pos::new(10.0, -5.0), Dir::new(10.0, -5.0));
    let line = Line::from_vector(vector, Colour::Red);
    assert_line_eq(
      line,
      Line::new(Colour::Red, Pos::new(10.0, -5.0), Pos::new(20.0, -10.0)),
    );
  }

  #[test]
  fn get_direction() {
    let line = Line::new(Colour::Red, Pos::new(10.0, -5.0), Pos::new(20.0, -10.0));
    let dir = line.direction();
    assert_dir_eq(dir, Dir::new(10.0, -5.0));
  }

  #[test]
  fn get_direction_from_inverted_coords() {
    let line = Line::new(Colour::Red, Pos::new(10.0, 5.0), Pos::new(20.0, 10.0));
    let dir = line.direction();
    assert_dir_eq(dir, Dir::new(-10.0, -5.0));
  }

  #[test]
  fn get_midpoint() {
    let line = Line::new(Colour::Red, Pos::new(10.0, -5.0), Pos::new(20.0, -10.0));
    let midpoint = line.midpoint();
    assert_pos_eq(midpoint, Pos::new(15.0, -7.5));
  }
}
