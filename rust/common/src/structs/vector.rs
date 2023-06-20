use std::{
  iter::Sum,
  ops::{Add, Mul, Neg},
};

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
