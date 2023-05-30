use cv::line::{Colour, Line, LineSegment, Point, Vector};
use Colour::{Black, Green, Orange, White, Yellow};

// The minimum length of an average line for it to be significant
const MINIMUM_LENGTH: f32 = 0.15;
// The slope at which the direction of line should flip
const DIRECTION_SLOPE: f32 = 0.25;

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
      let threshold: Line = Line::from_dir(Vector::new(sign, DIRECTION_SLOPE));
      if lies_on_right(&Point::from_vector(line.direction()), &threshold) {
        line.direction() * sign
      } else {
        -line.direction() * sign
      }
    })
    .sum();
  if weighted_dir.length() < MINIMUM_LENGTH {
    return None;
  }

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

/// Checks if `point` lies on the right of `line`
pub fn lies_on_right(point: &Point, line: &Line) -> bool {
  point.x > line.origin.x + (point.y - line.origin.y) / line.slope()
}

/// Returns all lines in `lines` that lie to the right of `boundary`
pub fn lines_on_right(lines: &[LineSegment], boundary: &Line) -> Vec<LineSegment> {
  lines
    .iter()
    .filter(|line| lies_on_right(&line.midpoint(), boundary))
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
  let yellow_line = get_average_line(lines, Yellow)
    .unwrap_or(Line::new(Point::new(0.0, 1.0), Vector::new(0.25, -1.0)));
  // Get all lines that lie right of the yellow line
  let right_lines: Vec<LineSegment> = lines_on_right(lines, &yellow_line);
  // Try to detect white line right of yellow line
  let white_line = get_average_line(&right_lines, White)
    .unwrap_or(Line::new(Point::new(1.0, 1.0), Vector::new(-0.25, -1.0)));
  // Do a bisection with yellow and white line
  let lane = bisect(&yellow_line, &white_line).unwrap_or({
    // If there is no intersection, both lines must have the same slope. Thus, we can simply
    // get the slope of one of the lines. Since there is no intersection, we can simply
    // estimate the centre of the lines by averaging their midpoints.
    let intersection = (yellow_line.origin + white_line.origin) / 2.0;
    Line::new(
      intersection - Point::from_vector(yellow_line.dir),
      yellow_line.dir * 2.0,
    )
  });

  Some(vec![
    LineSegment::from_line(lane, Green),
    LineSegment::from_line(yellow_line, Orange),
    LineSegment::from_line(white_line, Black),
  ])
}

#[cfg(test)]
mod tests {
  // TODO: add tests for detect_lane
}
