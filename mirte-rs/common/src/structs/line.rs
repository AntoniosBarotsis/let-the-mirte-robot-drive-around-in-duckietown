use crate::geometry_msgs::{Point, Vector3};
use crate::mirte_duckietown_msgs::{Line, LineSegment};

impl Line {
  pub fn new(origin: Point, direction: Vector3) -> Self {
    Self { origin, direction }
  }

  pub fn from_line(line: &LineSegment) -> Self {
    Self {
      origin: line.start,
      direction: line.direction(),
    }
  }

  pub fn from_vec(direction: Vector3) -> Self {
    Self {
      origin: Point::ORIGIN,
      direction,
    }
  }

  pub fn slope(&self) -> f64 {
    self.direction.y / self.direction.x
  }

  pub fn clamped_slope(&self) -> f64 {
    let slope = self.slope();
    if slope.is_nan() || slope.is_infinite() || slope > 100_000_000_000.0 {
      100_000_000_000.0
    } else {
      slope
    }
  }

  /// Returns the y value at the given x value on the vector
  fn y(&self, x: f64) -> f64 {
    self.slope() * (x - self.origin.x) + self.origin.y
  }

  pub fn intersect(&self, other: &Line) -> Option<Point> {
    // We clamp the slopes to still get a valid output even if the slopes are infinite or NaN
    let slope1 = self.clamped_slope();
    let slope2 = other.clamped_slope();
    // If slopes are the same, x will be invalid because the lines never intersect.
    if (slope1 - slope2).abs() < f64::EPSILON {
      return None;
    }

    let x = (self.origin.x * slope1 - self.origin.y - other.origin.x * slope2 + other.origin.y)
      / (slope1 - slope2);
    let y = if self.slope().is_infinite() {
      other.y(x)
    } else {
      self.y(x)
    };

    Some(Point::new(x, y))
  }
}

impl Copy for Line {}

impl Line {
  pub fn line_eq(&self, other: &Self) -> bool {
    self.origin.point_eq(&other.origin) && self.direction.vec_eq(&other.direction)
  }
}

#[cfg(test)]
pub mod test {
  use crate::{
    float_eq,
    geometry_msgs::{Point, Vector3},
    mirte_msgs::Line,
  };

  #[test]
  fn create_line() {
    let line = Line::new(Point::new(6.5, 3.0), Vector3::new(1.0, 1.0));
    assert!(line.origin.point_eq(&Point::new(6.5, 3.0)));
    assert!(line.direction.vec_eq(&Vector3::new(1.0, 1.0)));
  }

  #[test]
  fn line_inf_slope() {
    let line = Line::new(Point::new(5.0, 0.0), Vector3::new(0.0, 10.0));
    let slope = line.slope();
    assert!(float_eq(slope, f64::INFINITY));
  }

  #[test]
  fn line_slope() {
    let line = Line::new(Point::new(5.0, 0.0), Vector3::new(1.0, 10.0));
    let slope = line.slope();
    assert!(float_eq(slope, 10.0));
  }

  #[test]
  #[allow(clippy::expect_used)]
  fn intersect() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector3::new(1.0, 1.0));
    let line2 = Line::new(Point::new(0.0, 0.0), Vector3::new(1.0, -1.0));
    let intersection = line1.intersect(&line2).expect("No intersection found!");
    assert!(intersection.point_eq(&Point::new(2.0, -2.0)));
  }

  #[test]
  #[allow(clippy::expect_used)]
  fn intersect_with_vertical() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector3::new(0.0, 1.0));
    let line2 = Line::new(Point::new(0.0, 0.0), Vector3::new(1.0, 1.0));
    let intersection = line1.intersect(&line2).expect("No intersection found!");
    println!("{intersection:?}");
    assert!(intersection.point_eq(&Point::new(4.0, 4.0)));
  }

  #[test]
  fn intersect_with_identical() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector3::new(1.0, 1.0));
    let line2 = Line::new(Point::new(4.0, 0.0), Vector3::new(1.0, 1.0));
    let intersection = line1.intersect(&line2);
    assert!(intersection.is_none());
  }

  #[test]
  fn intersect_with_same_slope() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector3::new(1.0, 1.0));
    let line2 = Line::new(Point::new(8.0, 0.0), Vector3::new(1.0, 1.0));
    let intersection = line1.intersect(&line2);
    assert!(intersection.is_none());
  }

  #[test]
  fn create_line_from_vector() {
    let vec = Vector3::new(10.0, -5.0);
    let line = Line::from_vec(vec);
    assert!(line.line_eq(&Line::new(Point::new(0.0, 0.0), Vector3::new(10.0, -5.0))),);
  }
}
