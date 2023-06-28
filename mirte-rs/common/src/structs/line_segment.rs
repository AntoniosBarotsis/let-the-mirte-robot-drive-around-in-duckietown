use crate::{
  geometry_msgs::{Point, Vector3},
  mirte_duckietown_msgs::{Line, LineSegment, LineSegmentList},
  IMAGE_CROP_HEIGHT,
};

use super::colour::ColourEnum;

impl LineSegment {
  pub fn new(colour: ColourEnum, start: Point, end: Point) -> Self {
    let colour = colour.into();
    Self { colour, start, end }
  }

  pub fn from_line(line: Line, colour: ColourEnum) -> Self {
    let upper_y = Line::new(
      Point::new(0.0, f64::from(IMAGE_CROP_HEIGHT)),
      Vector3::new(1.0, 0.0),
    );
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

  pub fn segment_eq(&self, other: &Self) -> bool {
    self.colour == other.colour
      && self.start.point_eq(&other.start)
      && self.end.point_eq(&other.end)
  }
}

impl Copy for LineSegment {}

impl From<Vec<LineSegment>> for LineSegmentList {
  fn from(value: Vec<LineSegment>) -> Self {
    let segments = value.into_iter().map(LineSegment::from).collect::<Vec<_>>();

    LineSegmentList { segments }
  }
}

#[cfg(test)]
pub mod test {
  use crate::{
    geometry_msgs::{Point, Vector3},
    mirte_msgs::{Line, LineSegment},
    structs::colour::ColourEnum,
  };

  #[test]
  fn create_line_segment() {
    let segment = LineSegment::new(
      ColourEnum::Red,
      Point::new(10.0, -5.0),
      Point::new(20.0, -10.0),
    );
    assert!(segment.segment_eq(&LineSegment {
      colour: ColourEnum::Red.into(),
      start: Point::new(10.0, -5.0),
      end: Point::new(20.0, -10.0)
    }));
  }

  #[test]
  fn create_line_segment_from_line() {
    let line = Line::new(Point::new(0.5, 1.0), Vector3::new(0.0, -1.0));
    let segment = LineSegment::from_line(line, ColourEnum::Red);
    assert!(segment.start.point_eq(&Point::new(0.5, 1.0)));
  }

  #[test]
  fn create_line_segment_from_horizontal_line() {
    let line = Line::new(Point::new(0.5, 0.75), Vector3::new(1.0, 0.0));
    let segment = LineSegment::from_line(line, ColourEnum::Red);
    assert!(segment.start.point_eq(&Point::new(0.0, 0.75)));
    assert!(segment.end.point_eq(&Point::new(1.0, 0.75)));
  }

  #[test]
  fn get_direction() {
    let segment = LineSegment::new(
      ColourEnum::Red,
      Point::new(10.0, -5.0),
      Point::new(20.0, -10.0),
    );
    let dir = segment.direction();
    assert!(dir.vec_eq(&Vector3::new(10.0, -5.0)));
  }

  #[test]
  fn get_direction_from_inverted_coords() {
    let segment = LineSegment::new(
      ColourEnum::Red,
      Point::new(10.0, 5.0),
      Point::new(20.0, 10.0),
    );
    let dir = segment.direction();
    assert!(dir.vec_eq(&Vector3::new(-10.0, -5.0)));
  }

  #[test]
  fn get_midpoint() {
    let segment = LineSegment::new(
      ColourEnum::Red,
      Point::new(10.0, -5.0),
      Point::new(20.0, -10.0),
    );
    let midpoint = segment.midpoint();
    assert!(midpoint.point_eq(&Point::new(15.0, -7.5)));
  }
}
