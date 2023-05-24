use cv::line::{Colour, Dir, Line, Pos, Vector};

fn get_average_line(lines: &[Line], colour: Colour) -> Option<Vector> {
  let coloured_lines: Vec<Line> = lines
    .iter()
    .filter(|line| line.colour == colour)
    .copied()
    .collect();
  if coloured_lines.is_empty() {
    return None;
  }
  let weighted_dir: Dir = coloured_lines.iter().map(Line::direction).sum();

  // Get average line position with line length as weight
  let total_squared_length: f32 = coloured_lines
    .iter()
    .map(|line| line.direction().squared_length())
    .sum();
  let average_midpoint: Pos = coloured_lines
    .iter()
    .fold(Pos { x: 0.0, y: 0.0 }, |pos, line| {
      pos + line.midpoint() * line.direction().squared_length()
    })
    / total_squared_length;

  Some(Vector {
    origin: average_midpoint - Pos::from_dir(weighted_dir) / 2.0,
    dir: weighted_dir,
  })
}

fn estimate_lane(line: Vector) -> Line {
  // For now, simply returning the given vector may be enough. This will only become clear once we
  // have working motor controls for Mirte based on the lane. If it doesn't work well, we may need
  // to revisit this function
  Line::from_vector(line, Colour::Green)
}

/// Checks if `pos` lies on the right of `vector`
pub fn lies_on_right(pos: &Pos, vector: &Vector) -> bool {
  pos.x > vector.origin.x + (pos.y - vector.end().x) / vector.slope()
}

/// Returns all lines in `lines` that lie to the right of `vector`
pub fn lines_on_right(lines: &[Line], vector: &Vector) -> Vec<Line> {
  lines
    .iter()
    .filter(|line| lies_on_right(&line.midpoint(), vector))
    .copied()
    .collect()
}

pub fn detect_lane(lines: &[Line]) -> Result<Vec<Line>, &'static str> {
  // Try to detect yellow line in image
  if let Some(y_vec) = get_average_line(lines, Colour::Yellow) {
    eprintln!("Yellow line found! Looking for white lines to the right of the yellow line...");
    // Get lines right of yellow line
    let right_lines: Vec<Line> = lines_on_right(lines, &y_vec);
    // Try to detect white line right of yellow line
    if let Some(w_vec) = get_average_line(&right_lines, Colour::White) {
      eprintln!("Right white line found! Trying to find intersection...");
      // Do a bisection with yellow and white line
      // Calculate slope of bisection line
      let lane = if let Some(intersection) = w_vec.intersect(&y_vec) {
        eprintln!("Intersection found! Calculating lane...");
        let y_dir = y_vec.dir;
        let w_dir = w_vec.dir;
        let dir = y_dir * w_dir.length() + w_dir * y_dir.length();
        Line::from_vector(
          Vector::new(intersection - Pos::from_dir(dir), dir * 2.0),
          Colour::Green,
        )
      } else {
        eprintln!(
          "No intersection found! Using average of line positions instead. Calculating lane..."
        );
        // If there is no intersection, both lines must have the same slope. Thus, we can simply
        // get the slope of one of the lines. Since there is no intersection, we can simply
        // estimate the centre of the lines by averaging their midpoints.
        let intersection = (y_vec.midpoint() + w_vec.midpoint()) / 2.0;
        Line::from_vector(
          Vector::new(intersection - Pos::from_dir(y_vec.dir), y_vec.dir * 2.0),
          Colour::Green,
        )
      };

      eprintln!("Lane calculated: {:?}, {:?}", lane.start, lane.end);
      Ok(vec![
        Line::from_vector(y_vec, Colour::Orange),
        Line::from_vector(w_vec, Colour::Black),
        lane,
      ])
    } else {
      eprintln!("No right white line found! Estimating lane based on yellow line...");
      // If no white line right of yellow line found, try to estimate lane based on yellow line
      Ok(vec![
        Line::from_vector(y_vec, Colour::Orange),
        estimate_lane(y_vec),
      ])
    }
  } else {
    eprintln!("No yellow line found! Looking for white lines in image...");
    // Get lines on right side of screen (640x480)
    // TODO: get middle of screen from cv crate
    let middle_vec = Vector::new(Pos::new(350.0, 0.0), Dir::new(0.0, 1000.0));
    let right_lines: Vec<Line> = lines_on_right(lines, &middle_vec);
    // If no yellow line found, try to detect white line in image
    if let Some(w_vec) = get_average_line(&right_lines, Colour::White) {
      eprintln!("White line found! Estimating lane based on white line...");
      return Ok(vec![
        Line::from_vector(w_vec, Colour::Black),
        estimate_lane(w_vec),
      ]);
    }
    eprintln!("No yellow or white lines found! Unable to detect lane.");
    // If no yellow or white lines found, return error
    Err("Unable to detect lane without yellow or white lines!")
  }
}
