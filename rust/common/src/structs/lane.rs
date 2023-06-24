use crate::mirte_duckietown_msgs::{Lane, Line, LineSegment};

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
