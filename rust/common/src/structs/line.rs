use crate::geometry_msgs::{Point, Vector3};
use crate::mirte_msgs::{Line, LineSegment};

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

  pub fn from_dir(direction: Vector3) -> Self {
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

    Some(Point { x, y, z: 0.0 })
  }
}

impl Copy for Line {}
