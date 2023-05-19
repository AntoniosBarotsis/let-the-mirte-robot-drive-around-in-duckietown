use std::iter::Sum;
use std::ops::{Add, Div, Sub};

// Represents a line color
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Colour {
  Yellow,
  White,
  Green,
}

// Represents a end coordinate of a line
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct Pos {
  pub x: i32,
  pub y: i32,
}

impl Pos {
  pub fn squared_distance(&self) -> i32 {
    self.x * self.x + self.y * self.y
  }

  // Commented to make it compile
  //pub fn distance(&self) -> f64 {
  //  f64::
  //}
}

impl Sub for Pos {
  type Output = Pos;

  fn sub(self, rhs: Self) -> Self::Output {
    Pos {
      x: self.x - rhs.x,
      y: self.y - rhs.y,
    }
  }
}

impl Add for Pos {
  type Output = Pos;

  fn add(self, rhs: Self) -> Self::Output {
    Pos {
      x: self.x + rhs.x,
      y: self.y + rhs.y,
    }
  }
}

impl Div<i32> for Pos {
  type Output = Pos;

  fn div(self, rhs: i32) -> Self::Output {
    Pos {
      x: self.x / rhs,
      y: self.y / rhs,
    }
  }
}

impl Sum for Pos {
  fn sum<I>(iter: I) -> Pos
  where
    I: Iterator<Item = Self>,
  {
    let mut sum = Pos { x: 0, y: 0 };
    for pos in iter {
      sum = pos + sum;
    }
    sum
  }
}

// Represents a line
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct Line {
  pub colour: Colour,
  pub pos1: Pos,
  pub pos2: Pos,
}

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_YELLOW: &[[u8; 3]; 2] = &[[25, 0, 0], [45, 255, 255]];
pub static HSV_WHITE: &[[u8; 3]; 2] = &[[0, 0, 150], [179, 60, 255]];
pub static HSV_GREEN: &[[u8; 3]; 2] = &[[45, 0, 0], [90, 255, 255]];

impl Line {
  /// Gets the direction of the line, expressed as a vector from (0,0) to the resulting position.
  pub fn direction(&self) -> Pos {
    if self.pos1.y < self.pos2.y {
      self.pos1 - self.pos2
    } else {
      self.pos2 - self.pos1
    }
  }

  pub fn slope(&self) -> f64 {
    let a = f64::from(self.pos1.x);
    let b = f64::from(self.pos1.y);
    let c = f64::from(self.pos2.x);
    let d = f64::from(self.pos2.y);
    let slope = (d - b) / (c - a);
    // If (c - a) is 0, the slope is invalid, and it means the line must be stictly vertical, which means simply
    // returning a high value suffices for the calculations we are doing.
    if slope.is_nan() {
      return 1000.0;
    }
    slope
  }

  pub fn intersect(&self, other: &Line) -> Option<Pos> {
    let slope1 = self.slope();
    let slope2 = other.slope();

    let a = f64::from(self.pos1.x);
    let b = f64::from(self.pos1.y);
    let e = f64::from(other.pos1.x);
    let f = f64::from(other.pos1.y);

    let x = (a * slope1 - b - e * slope2 + f) / (slope1 - slope2);
    // If (slope1 - slope2) is 0, x will be invalid because the lines never intersect.
    if x.is_nan() {
      return None;
    }
    let y = slope1 * (x - a) + b;

    Some(Pos {
      //TODO: Safely convert f64 to i32 to avoid panicing
      x: x as i32,
      y: y as i32,
    })
  }
}
