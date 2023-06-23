use crate::geometry_msgs::{Point, Vector3};
use crate::mirte_msgs::{Line, LineSegment, LineSegmentList};

use super::colour::ColourEnum;

pub const CROP_HEIGHT: f64 = 0.58;

impl LineSegment {
  pub fn new(colour: ColourEnum, start: Point, end: Point) -> Self {
    let colour = colour.into();
    Self { colour, start, end }
  }

  pub fn from_line(line: Line, colour: ColourEnum) -> Self {
    let upper_y = Line::new(Point::new(0.0, CROP_HEIGHT), Vector3::new(1.0, 0.0));
    let lower_y = Line::new(Point::new(0.0, 1.0), Vector3::new(1.0, 0.0));

    let upper_y_intersection = line
      .intersect(&upper_y)
      .unwrap_or(Point::new(1.0, line.origin.y));
    let lower_y_intersection = line
      .intersect(&lower_y)
      .unwrap_or(Point::new(0.0, line.origin.y));

    Self::new(colour, lower_y_intersection, upper_y_intersection)
  }

  pub fn direction(&self) -> Vector3 {
    if self.start.y < self.end.y {
      Vector3::from_point(self.start - self.end)
    } else {
      Vector3::from_point(self.end - self.start)
    }
  }

  pub fn midpoint(&self) -> Point {
    (self.start + self.end) / 2.0
  }
}

impl Copy for LineSegment {}

impl From<Vec<LineSegment>> for LineSegmentList {
  fn from(value: Vec<LineSegment>) -> Self {
    let segments = value.into_iter().map(LineSegment::from).collect::<Vec<_>>();

    LineSegmentList { segments }
  }
}
