use std::iter::Sum;
use std::ops::{Add, Sub};

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
}
