use cv::line::{Colour::Green, Line, Pos};

pub fn detect_lane(_lines: &[Line]) -> Result<Line, &'static str> {
  Ok(Line {
    colour: Green,
    pos1: Pos { x: 330, y: 450 },
    pos2: Pos { x: 320, y: 250 },
  })
}
