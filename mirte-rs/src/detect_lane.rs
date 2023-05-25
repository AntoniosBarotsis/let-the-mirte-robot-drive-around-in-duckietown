use cv::line::{Colour, Dir, Line, Pos, Vector};

/// Averages all lines with a given colour weighted by their length
fn get_average_line(lines: &[Line], colour: Colour) -> Option<Vector> {
  let coloured_lines: Vec<Line> = lines
    .iter()
    .filter(|line| line.colour == colour)
    .copied()
    .collect();
  if coloured_lines.is_empty() {
    return None;
  }
  //let weighted_dir: Dir = coloured_lines.iter().map(Line::direction).sum();
  println!("Detecting {colour:?} lines");
  let double_dir: Dir = coloured_lines
    .iter()
    .map(|line| {
      let dir = line.direction();
      let length = dir.length();
      let n_dir = dir / length;
      Dir::new(
        2.0 * n_dir.x * n_dir.y,
        n_dir.x * n_dir.x - n_dir.y * n_dir.y,
      )
    })
    .sum();
  let degree = double_dir.x.atan2(double_dir.y) / 2.0;
  let weighted_dir = Dir::new(degree.cos(), degree.sin()) * 100.0;

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

/// Estimates the lane given a line, which should either be a yellow or white line from Duckietown.
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

/// Returns the bisection between `vec1` and `vec2`, if it exists
pub fn bisect(vec1: &Vector, vec2: &Vector) -> Option<Vector> {
  vec2.intersect(vec1).map(|intersection| {
    let dir1 = vec1.dir;
    let dir2 = vec2.dir;
    let dir = dir1 * dir2.length() + dir2 * dir1.length();
    Vector::new(intersection - Pos::from_dir(dir), dir * 2.0)
  })
}

/// Detects the lane based on given line segments. Returns a `Line` if successful.
pub fn detect_lane(lines: &[Line]) -> Option<Line> {
  detect_lane_debug(lines).map(|lines| lines[0])
}

/// Detects the lane based on given line segments. Returns a `Vec<Line>` if successful, where the
/// first line is the lane, and any other lines are debug information for rendering to the screen.
/// For example, if it detects a yellow and/or white line, it'll add those lines to be rendered.
pub fn detect_lane_debug(lines: &[Line]) -> Option<Vec<Line>> {
  // Try to detect yellow line in image
  if let Some(y_vec) = get_average_line(lines, Colour::Yellow) {
    // Get all lines that lie right of the yellow line
    let right_lines: Vec<Line> = lines_on_right(lines, &y_vec);
    // Try to detect white line right of yellow line
    if let Some(w_vec) = get_average_line(&right_lines, Colour::White) {
      // Do a bisection with yellow and white line
      let lane = bisect(&y_vec, &w_vec).unwrap_or({
        // If there is no intersection, both lines must have the same slope. Thus, we can simply
        // get the slope of one of the lines. Since there is no intersection, we can simply
        // estimate the centre of the lines by averaging their midpoints.
        let intersection = (y_vec.midpoint() + w_vec.midpoint()) / 2.0;
        Vector::new(intersection - Pos::from_dir(y_vec.dir), y_vec.dir * 2.0)
      });

      Some(vec![
        Line::from_vector(lane, Colour::Green),
        Line::from_vector(y_vec, Colour::Orange),
        Line::from_vector(w_vec, Colour::Black),
      ])
    } else {
      // If no white line right of yellow line found, try to estimate lane based on yellow line
      Some(vec![
        estimate_lane(y_vec),
        Line::from_vector(y_vec, Colour::Orange),
      ])
    }
  } else {
    // If no yellow line is found, it's probably in a turn where the yellow lines are out of view.
    // In that case, we can look for a white line on the right of the screen and estimate the lane
    // using it.
    // Get lines on right side of screen (640x480)
    // TODO: get middle of screen from cv crate
    let middle_vec = Vector::new(Pos::new(350.0, 0.0), Dir::new(0.0, 1000.0));
    let right_lines: Vec<Line> = lines_on_right(lines, &middle_vec);
    // If no yellow line found, try to detect white line and estimate the lane based on that.
    // Returns `None` if no white line found.
    get_average_line(&right_lines, Colour::White).map(|w_vec| {
      vec![
        estimate_lane(w_vec),
        Line::from_vector(w_vec, Colour::Black),
      ]
    })
  }
}
