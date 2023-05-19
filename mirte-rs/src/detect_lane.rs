use cv::line::{Colour, Line, Pos};

const THRESHOLD: i32 = 100000;

pub fn get_average_line(lines: &[Line]) -> Option<Line> {
  let direction: Pos = lines.iter().map(|line| line.direction()).sum();
  println!("{}", direction.squared_distance());
  if direction.squared_distance() < THRESHOLD {
    return None;
  }
  // Get the point with the highest y value (which is closest to the bottom of the screen)
  let lowest_point: Pos = lines
    .iter()
    .flat_map(|line| vec![line.pos1, line.pos2])
    .max_by_key(|pos| pos.y)
    .unwrap();
  println!(
    "pos1: {:?}, pos2: {:?}",
    lowest_point,
    lowest_point + direction
  );
  Some(Line {
    colour: Colour::White,
    pos1: lowest_point,
    pos2: lowest_point + direction,
  })
}

pub fn detect_lane(lines: &[Line]) -> Result<Line, &'static str> {
  let yellow_lines: Vec<Line> = lines
    .iter()
    .filter(|line| line.colour == Colour::Yellow)
    .copied()
    .collect();
  let yellow_line = get_average_line(&yellow_lines);
  match yellow_line {
    Some(line) => (),
    None => (),
  }

  let line = yellow_line.unwrap();
  Ok(Line {
    colour: Colour::Green,
    pos1: line.pos1,
    pos2: line.pos2,
  })
  //yellow_line.ok_or("Error")
}
