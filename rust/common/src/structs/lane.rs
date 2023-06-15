use crate::mirte_msgs::{Colour, Lane, Line, LineSegment};

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
    lane_colour: Colour,
    left_colour: Colour,
    right_colour: Colour,
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
