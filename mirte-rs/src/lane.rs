use cv::line::{Colour, Line, LineSegment};

#[derive(Debug, Clone, Copy)]
pub struct Lane {
  pub lane: Line,
  pub left_line: Line,
  pub right_line: Line,
}

impl Lane {
  pub fn new(lane: Line, left_line: Line, right_line: Line) -> Self {
    Self {
      lane,
      left_line,
      right_line,
    }
  }

  pub fn get_coloured_segments(
    &self,
    lane_colour: Colour,
    left_colour: Colour,
    right_colour: Colour,
  ) -> Vec<LineSegment> {
    [
      (self.lane, lane_colour),
      (self.left_line, left_colour),
      (self.right_line, right_colour),
    ]
    .iter()
    .map(|&(line, color)| LineSegment::from_line(line, color))
    .collect()
  }
}
