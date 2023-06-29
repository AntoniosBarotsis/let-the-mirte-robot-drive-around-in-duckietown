use std::ops::{Add, Div, Mul, Sub};

use crate::float_eq;
use crate::geometry_msgs::{Point, Vector3};

impl Point {
  pub const fn new(x: f64, y: f64) -> Self {
    Self { x, y, z: 0.0 }
  }

  pub fn from_vector(vec: Vector3) -> Self {
    Self::new(vec.x, vec.y)
  }

  pub fn point_eq(&self, other: &Self) -> bool {
    float_eq(self.x, other.x) && float_eq(self.y, other.y) && float_eq(self.z, other.z)
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

#[cfg(test)]
mod test {
  use crate::{
    float_eq,
    geometry_msgs::{Point, Vector3},
  };

  #[test]
  fn create_point() {
    let point = Point::new(10.0, -5.0);
    assert!(float_eq(point.x, 10.0));
    assert!(float_eq(point.y, -5.0));
  }

  #[test]
  fn create_point_from_vec() {
    let vec = Vector3::new(10.0, -5.0);
    let point = Point::from_vector(vec);
    assert!(point.point_eq(&Point::new(10.0, -5.0)));
  }
}
