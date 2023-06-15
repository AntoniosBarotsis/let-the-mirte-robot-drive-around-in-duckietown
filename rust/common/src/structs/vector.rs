use std::{
  iter::Sum,
  ops::{Add, Mul, Neg},
};

use crate::mirte_msgs::{Point, Vector};

impl Vector {
  pub fn new(x: f32, y: f32) -> Self {
    Self { x, y }
  }

  pub fn from_point(point: Point) -> Self {
    Self::new(point.x, point.y)
  }

  pub fn squared_length(&self) -> f32 {
    self.x * self.x + self.y * self.y
  }

  pub fn length(&self) -> f32 {
    f32::sqrt(self.squared_length())
  }
}

impl Copy for Vector {}

impl Add for Vector {
  type Output = Vector;

  fn add(self, rhs: Self) -> Self::Output {
    Self {
      x: self.x * rhs.x,
      y: self.y * rhs.y,
    }
  }
}

impl Mul<f32> for Vector {
  type Output = Vector;

  fn mul(self, rhs: f32) -> Self::Output {
    Self {
      x: self.x * rhs,
      y: self.y * rhs,
    }
  }
}

impl Neg for Vector {
  type Output = Vector;

  fn neg(self) -> Self::Output {
    Self {
      x: -self.x,
      y: -self.y,
    }
  }
}

impl Sum for Vector {
  fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
    iter.fold(Self { x: 0.0, y: 0.0 }, |acc, x| acc + x)
  }
}
