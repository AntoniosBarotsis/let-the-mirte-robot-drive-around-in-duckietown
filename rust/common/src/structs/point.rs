use std::ops::{Add, Div, Mul, Sub};

use crate::geometry_msgs::{Point, Vector3};

impl Point {
  pub const fn new(x: f64, y: f64) -> Self {
    Self { x, y, z: 0.0 }
  }

  pub fn from_vector(vec: Vector3) -> Self {
    Self::new(vec.x, vec.y)
  }

  pub const ORIGIN: Point = Point::new(0.0, 0.0);
}

impl Add for Point {
  type Output = Point;

  fn add(self, rhs: Self) -> Self::Output {
    Self::new(self.x + rhs.x, self.y + rhs.y)
  }
}

impl Sub for Point {
  type Output = Point;

  fn sub(self, rhs: Self) -> Self::Output {
    Self::new(self.x - rhs.x, self.y - rhs.y)
  }
}

impl Div<f64> for Point {
  type Output = Point;
  fn div(self, rhs: f64) -> Self::Output {
    Self::new(self.x / rhs, self.y / rhs)
  }
}

impl Copy for Point {}

impl Mul<f64> for Point {
  type Output = Point;

  fn mul(self, rhs: f64) -> Self::Output {
    Self::new(self.x * rhs, self.y * rhs)
  }
}
