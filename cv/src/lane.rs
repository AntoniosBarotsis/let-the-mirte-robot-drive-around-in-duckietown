use crate::line::{Colour, Line, LineSegment};

#[derive(Debug, Clone, Copy)]
pub struct Lane {
  pub centre: Line,
  pub left: Line,
  pub right: Line,
}

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
