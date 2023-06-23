use crate::mirte_msgs::{Lane, Line, LineSegment};

use super::colour::ColourEnum;

impl Lane {
  pub fn new(lane: Line, left_line: Line, right_line: Line) -> Self {
    Self {
      centre: lane,
      left: left_line,
      right: right_line,
    }
  }

  pub fn get_coloured_segments(
    &self,
    lane_colour: ColourEnum,
    left_colour: ColourEnum,
    right_colour: ColourEnum,
  ) -> Vec<LineSegment> {
    [
      (self.centre, lane_colour),
      (self.left, left_colour),
      (self.right, right_colour),
    ]
    .iter()
    .map(|&(line, color)| LineSegment::from_line(line, color))
    .collect()
  }
}

impl Copy for Lane {}

#[cfg(test)]
mod test {
  use crate::{
    geometry_msgs::{Point, Vector3},
    mirte_msgs::{Lane, Line, LineSegment},
    structs::colour::ColourEnum,
  };

  #[test]
  fn test_get_coloured_segments() {
    let centre = Line::new(Point::new(0.0, 0.0), Vector3::new(0.0, 1.0));
    let left = Line::new(Point::new(-1.0, 0.0), Vector3::new(0.0, 1.0));
    let right = Line::new(Point::new(1.0, 0.0), Vector3::new(0.0, 1.0));
    let lane = Lane::new(centre, left, right);
    let segments = lane.get_coloured_segments(ColourEnum::Red, ColourEnum::Green, ColourEnum::Blue);
    assert!(segments[0].segment_eq(&LineSegment::from_line(centre, ColourEnum::Red)));
    assert!(segments[1].segment_eq(&LineSegment::from_line(left, ColourEnum::Green)));
    assert!(segments[2].segment_eq(&LineSegment::from_line(right, ColourEnum::Blue)));
  }
}
