use std::ops::{Add, Div, Mul, Sub};

use crate::mirte_msgs::{Point, Vector};

impl Point {
  pub fn new(x: f32, y: f32) -> Self {
    Self { x, y }
  }

  pub fn from_vector(vec: Vector) -> Self {
    Self::new(vec.x, vec.y)
  }

  pub const ORIGIN: Point = Point { x: 0.0, y: 0.0 };
}

impl Add for Point {
  type Output = Point;

  fn add(self, rhs: Self) -> Self::Output {
    Self {
      x: self.x + rhs.x,
      y: self.y + rhs.y,
    }
  }
}

impl Sub for Point {
  type Output = Point;

  fn sub(self, rhs: Self) -> Self::Output {
    Self {
      x: self.x - rhs.x,
      y: self.y - rhs.y,
    }
  }
}

impl Div<f32> for Point {
  type Output = Point;
  fn div(self, rhs: f32) -> Self::Output {
    Self {
      x: self.x / rhs,
      y: self.y / rhs,
    }
  }
}

impl Copy for Point {}

impl Mul<f32> for Point {
  type Output = Point;

  fn mul(self, rhs: f32) -> Self::Output {
    Self {
      x: self.x * rhs,
      y: self.y * rhs,
    }
  }
}
