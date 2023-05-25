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
  // TODO: use relative screen coords when implemented (for now, assuming resolution is 640x480)
  let y_vec = get_average_line(lines, Colour::Yellow)
    .unwrap_or(Vector::new(Pos::new(0.0, 480.0), Dir::new(500.0, -500.0)));
  // Get all lines that lie right of the yellow line
  let right_lines: Vec<Line> = lines_on_right(lines, &y_vec);
  // Try to detect white line right of yellow line
  let w_vec = get_average_line(&right_lines, Colour::White).unwrap_or(Vector::new(
    Pos::new(640.0, 480.0),
    Dir::new(-500.0, -500.0),
  ));
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
}
