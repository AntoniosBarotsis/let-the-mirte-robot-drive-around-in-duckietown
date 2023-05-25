use cv::line::{Colour, Dir, Line, Pos, Vector};
use Colour::{Black, Green, Orange, White, Yellow};

/// The slope that determines the direction of line segments
const DIRECTION_THRESHOLD: f32 = 0.1;

/// Averages all lines with a given colour weighted by their length
fn get_average_line(lines: &[Line], colour: Colour) -> Option<Vector> {
  // Get lines with given colour
  let coloured_lines: Vec<Line> = lines
    .iter()
    .filter(|line| line.colour == colour)
    .copied()
    .collect();
  if coloured_lines.is_empty() {
    return None;
  }

  // Get average line with line length as weight
  let weighted_dir: Dir = coloured_lines
    .iter()
    .map(|line| -> Dir {
      let sign: f32 = if line.colour == White { -1.0 } else { 1.0 };
      let threshold: Vector = Vector::from_dir(Dir::new(sign, DIRECTION_THRESHOLD));
      if lies_on_right(&Pos::from_dir(line.direction()), &threshold) {
        line.direction() * sign
      } else {
        -line.direction() * sign
      }
    })
    .sum();

  // Get average line position with line length as weight
  let total_squared_length: f32 = coloured_lines
    .iter()
    .map(|line| line.direction().squared_length())
    .sum();
  let avg_pos: Pos = coloured_lines.iter().fold(Pos::ORIGIN, |pos, line| {
    pos + line.midpoint() * line.direction().squared_length()
  }) / total_squared_length;

  Some(Vector {
    origin: avg_pos - Pos::from_dir(weighted_dir) / 2.0,
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
  let y_vec = get_average_line(lines, Yellow)
    .unwrap_or(Vector::new(Pos::new(0.0, 480.0), Dir::new(500.0, -500.0)));
  // Get all lines that lie right of the yellow line
  let right_lines: Vec<Line> = lines_on_right(lines, &y_vec);
  // Try to detect white line right of yellow line
  let w_vec = get_average_line(&right_lines, White).unwrap_or(Vector::new(
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
    Line::from_vector(lane, Green),
    Line::from_vector(y_vec, Orange),
    Line::from_vector(w_vec, Black),
  ])
}

#[cfg(test)]
mod tests {
  // TODO: add tests for detect_lane
}
