use cv::line::{Colour, Line, LineSegment, Point, Vector};
use Colour::{Black, Green, Orange, White, Yellow};

/// The slope that determines the direction of line segments
const DIRECTION_THRESHOLD: f32 = 0.1;

/// Averages all lines with a given colour weighted by their length
fn get_average_line(lines: &[LineSegment], colour: Colour) -> Option<Line> {
  // Get lines with given colour
  let coloured_lines: Vec<LineSegment> = lines
    .iter()
    .filter(|line| line.colour == colour)
    .copied()
    .collect();
  if coloured_lines.is_empty() {
    return None;
  }

  // Get average line with line length as weight
  let weighted_dir: Vector = coloured_lines
    .iter()
    .map(|line| -> Vector {
      let sign: f32 = if line.colour == White { -1.0 } else { 1.0 };
      let threshold: Line = Line::from_dir(Vector::new(sign, DIRECTION_THRESHOLD));
      if lies_on_right(&Point::from_vector(line.direction()), &threshold) {
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
  let avg_pos: Point = coloured_lines.iter().fold(Point::ORIGIN, |pos, line| {
    pos + line.midpoint() * line.direction().squared_length()
  }) / total_squared_length;

  Some(Line {
    origin: avg_pos - Point::from_vector(weighted_dir) / 2.0,
    dir: weighted_dir,
  })
}

/// Checks if `pos` lies on the right of `vector`
pub fn lies_on_right(pos: &Point, line: &Line) -> bool {
  pos.x > line.origin.x + (pos.y - line.origin.y) / line.slope()
}

/// Returns all lines in `lines` that lie to the right of `vector`
pub fn lines_on_right(lines: &[LineSegment], vector: &Line) -> Vec<LineSegment> {
  lines
    .iter()
    .filter(|line| lies_on_right(&line.midpoint(), vector))
    .copied()
    .collect()
}

/// Returns the bisection between `line1` and `line2`, if it exists
pub fn bisect(line1: &Line, line2: &Line) -> Option<Line> {
  line2.intersect(line1).map(|intersection| {
    let dir1 = line1.dir;
    let dir2 = line2.dir;
    let dir = dir1 * dir2.length() + dir2 * dir1.length();
    Line::new(intersection - Point::from_vector(dir), dir * 2.0)
  })
}

/// Detects the lane based on given line segments. Returns a `Line` if successful.
pub fn detect_lane(lines: &[LineSegment]) -> Option<LineSegment> {
  detect_lane_debug(lines).map(|lines| lines[0])
}

/// Detects the lane based on given line segments. Returns a `Vec<Line>` if successful, where the
/// first line is the lane, and any other lines are debug information for rendering to the screen.
/// For example, if it detects a yellow and/or white line, it'll add those lines to be rendered.
pub fn detect_lane_debug(lines: &[LineSegment]) -> Option<Vec<LineSegment>> {
  // Try to detect yellow line in image
  let y_vec = get_average_line(lines, Yellow)
    .unwrap_or(Line::new(Point::new(0.0, 1.0), Vector::new(0.25, -1.0)));
  // Get all lines that lie right of the yellow line
  let right_lines: Vec<LineSegment> = lines_on_right(lines, &y_vec);
  // Try to detect white line right of yellow line
  let w_vec = get_average_line(&right_lines, White)
    .unwrap_or(Line::new(Point::new(1.0, 1.0), Vector::new(-0.25, -1.0)));
  // Do a bisection with yellow and white line
  let lane = bisect(&y_vec, &w_vec).unwrap_or({
    // If there is no intersection, both lines must have the same slope. Thus, we can simply
    // get the slope of one of the lines. Since there is no intersection, we can simply
    // estimate the centre of the lines by averaging their midpoints.
    let intersection = (y_vec.origin + w_vec.origin) / 2.0;
    Line::new(
      intersection - Point::from_vector(y_vec.dir),
      y_vec.dir * 2.0,
    )
  });

  Some(vec![
    LineSegment::from_line(lane, Green),
    LineSegment::from_line(y_vec, Orange),
    LineSegment::from_line(w_vec, Black),
  ])
}

#[cfg(test)]
mod tests {
  // TODO: add tests for detect_lane
}
