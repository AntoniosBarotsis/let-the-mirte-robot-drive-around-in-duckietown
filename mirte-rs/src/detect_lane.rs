use cv::line::{Colour, Line, Pos};

const THRESHOLD: i32 = 100_000;

fn get_average_line(lines: &[Line], colour: Colour) -> Option<Line> {
  let coloured_lines: Vec<Line> = lines
    .iter()
    .filter(|line| line.colour == colour)
    .copied()
    .collect();
  let direction: Pos = coloured_lines.iter().map(cv::line::Line::direction).sum();
  println!("{}", direction.squared_distance());
  if direction.squared_distance() < THRESHOLD {
    return None;
  }
  // Get the point with the highest y value (which is closest to the bottom of the screen)
  let lowest_point: Pos = coloured_lines
    .iter()
    .flat_map(|line| vec![line.pos1, line.pos2])
    .max_by_key(|pos| pos.y)
    .expect("No lines found in image");
  Some(Line {
    colour: Colour::White,
    pos1: lowest_point,
    pos2: lowest_point + direction,
  })
}

fn estimate_lane(_line: &Line) -> Line {
  // TODO: Estimate line
  Line {
    colour: Colour::Green,
    pos1: Pos { x: 0, y: 0 },
    pos2: Pos { x: 100, y: 100 },
  }
}

/// Checks if `right_line` lies on the right of `left_line`
pub fn lies_on_right(left_line: &Line, right_line: &Line) -> bool {
  let right_line_midpoint = (right_line.pos1 + right_line.pos2) / 2;

  let a = f64::from(left_line.pos1.x);
  let c = f64::from(left_line.pos2.x);

  let left_line_x = a + (f64::from(right_line_midpoint.y) - c) / left_line.slope();

  f64::from(right_line_midpoint.x) > left_line_x
}

pub fn detect_lane(lines: &[Line]) -> Result<Vec<Line>, &'static str> {
  // Try to detect yellow line in image
  if let Some(yellow_line) = get_average_line(lines, Colour::Yellow) {
    // Get lines right of yellow line
    let right_lines: Vec<Line> = lines
      .iter()
      .filter(|line| lies_on_right(&yellow_line, line))
      .copied()
      .collect();
    // Try to detect white line right of yellow line
    if let Some(right_white_line) = get_average_line(&right_lines, Colour::White) {
      // Do a bisection with yellow and white line
      // Calculate slope of bisection line
      let slope = if let Some(intersection) = right_white_line.intersect(&yellow_line) {
      } else {
        //yellow_line.direction()
      };

      Ok(vec![Line {
        colour: Colour::Green,
        pos1: right_white_line.pos1,
        pos2: right_white_line.pos2,
      }])
    } else {
      // If no white line right of yellow line found, try to estimate lane based on yellow line
      Ok(vec![estimate_lane(&yellow_line)])
    }
  } else {
    // If no yellow line found, try to detect white line in image
    if let Some(white_line) = get_average_line(lines, Colour::White) {
      return Ok(vec![estimate_lane(&white_line)]);
    }
    // If no yellow or white lines found, return error
    Err("Unable to detect lane without yellow or white lines!")
  }
}
