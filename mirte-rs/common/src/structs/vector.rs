use std::{
  iter::Sum,
  ops::{Add, Mul, Neg},
};

use crate::float_eq;
use crate::geometry_msgs::{Point, Vector3};

impl Vector3 {
  pub const fn new(x: f64, y: f64) -> Self {
    Self { x, y, z: 0.0 }
  }

  pub fn from_point(point: Point) -> Self {
    Self::new(point.x, point.y)
  }

  pub fn squared_length(&self) -> f64 {
    self.x * self.x + self.y * self.y
  }

  pub fn length(&self) -> f64 {
    f64::sqrt(self.squared_length())
  }

  pub fn vec_eq(&self, other: &Self) -> bool {
    float_eq(self.x, other.x) && float_eq(self.y, other.y) && float_eq(self.z, other.z)
  }
}

impl Copy for Vector3 {}

impl Add for Vector3 {
  type Output = Vector3;

  fn add(self, rhs: Self) -> Self::Output {
    Self::new(self.x + rhs.x, self.y + rhs.y)
  }
}

impl Mul<f64> for Vector3 {
  type Output = Vector3;

  fn mul(self, rhs: f64) -> Self::Output {
    Self::new(self.x * rhs, self.y * rhs)
  }
}

impl Neg for Vector3 {
  type Output = Vector3;

  fn neg(self) -> Self::Output {
    Self::new(-self.x, -self.y)
  }
}

impl Sum for Vector3 {
  fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
    iter.fold(Self::new(0.0, 0.0), |acc, x| acc + x)
  }
}

#[cfg(test)]
pub mod test {
  use crate::{
    float_eq,
    geometry_msgs::{Point, Vector3},
  };

  #[test]
  fn create_vec() {
    let vec = Vector3::new(10.0, -5.0);
    assert!(vec.vec_eq(&Vector3::new(10.0, -5.0)));
  }

  #[test]
  fn create_vec_from_point() {
    let point = Point::new(10.0, -5.0);
    let vec = Vector3::from_point(point);
    assert!(vec.vec_eq(&Vector3::new(10.0, -5.0)));
  }

  #[test]
  fn compute_squared_length() {
    let vec = Vector3::new(10.0, -5.0);
    assert!(float_eq(vec.squared_length(), 125.0));
  }

  #[test]
  fn compute_length() {
    let vec = Vector3::new(3.0, -4.0);
    assert!(float_eq(vec.length(), 5.0));
  }
}
