use cv::line::{Colour::Yellow, Line, Pos};

pub fn detect_lane(_lines: &[Line]) -> Result<Line, &'static str> {
  Ok(Line {
    colour: Yellow,
    pos1: Pos { x: 100, y: 0 },
    pos2: Pos { x: 100, y: 100 },
  })
}
