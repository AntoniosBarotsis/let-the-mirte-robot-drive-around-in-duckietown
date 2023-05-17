use cv::line::{Colour::Yellow, Line, Pos};

pub fn detect_lane(lines: &Vec<Line>) -> Result<Line, &'static str> {
  Ok(Line {
    colour: Yellow,
    pos1: Pos { x: 100.0, y: 0.0 },
    pos2: Pos { x: 100.0, y: 100.0 },
  })
}
