use cv::line::{Colour, Lane, Line, LineSegment, Point, Vector};
use Colour::{White, Yellow};

// The minimum length of an average line for it to be significant
const MINIMUM_LENGTH: f32 = 0.15;
// The slope at which the direction of line should flip
const DIRECTION_SLOPE: f32 = 0.25;

// Default lines for when no lines are found
const DEFAULT_YELLOW_LINE: Line = Line {
  origin: Point { x: 0.0, y: 1.0 },
  dir: Vector { x: 0.25, y: -1.0 },
};
const DEFAULT_WHITE_LINE: Line = Line {
  origin: Point { x: 1.0, y: 1.0 },
  dir: Vector { x: -0.25, y: -1.0 },
};

/// Averages all lines with a given colour weighted by their length
fn get_average_line(lines: &[LineSegment], colour: Colour) -> Option<Line> {
  // Get lines with given colour
  let coloured_lines: Vec<LineSegment> = lines
    .iter()
    .filter(|line| line.colour == colour)
    .copied()
    .collect();

  // Get average line with line length as weight
  let weighted_dir: Vector = coloured_lines
    .iter()
    .map(|line| -> Vector {
      let sign: f32 = if line.colour == White { -1.0 } else { 1.0 };
      let threshold: Line = Line::from_dir(Vector::new(sign, DIRECTION_SLOPE));
      if lies_on_right(Point::from_vector(line.direction()), &threshold) {
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
    origin: avg_pos,
    dir: weighted_dir,
  })
}

/// Checks if `point` lies on the right of `boundary`
fn lies_on_right(point: Point, boundary: &Line) -> bool {
  point.x > boundary.origin.x + (point.y - boundary.origin.y) / boundary.slope()
}

/// Returns all lines in `lines` that lie to the right of `boundary`
fn lines_on_right(lines: &[LineSegment], boundary: &Line) -> Vec<LineSegment> {
  lines
    .iter()
    .filter(|line| lies_on_right(line.midpoint(), boundary))
    .copied()
    .collect()
}

/// Returns the bisection between `line1` and `line2`, if it exists
fn bisect(line1: &Line, line2: &Line) -> Option<Line> {
  line2.intersect(line1).map(|intersection| {
    let dir1 = line1.dir;
    let dir2 = line2.dir;
    let dir = dir1 * dir2.length() + dir2 * dir1.length();
    Line::new(intersection - Point::from_vector(dir), dir)
  })
}

/// Returns the midline between `line1` and `line2`, or a default midline if they don't intersect
fn get_midline(line1: &Line, line2: &Line) -> Line {
  bisect(line1, line2).unwrap_or({
    let midpoint = (line1.origin + line2.origin) / 2.0;
    Line::new(midpoint, line1.dir)
  })
}

/// Detects the lane based on given line segments. Returns a `Lane` if successful.
pub fn detect_lane(lines: &[LineSegment]) -> Lane {
  let yellow_line = get_average_line(lines, Yellow).unwrap_or(DEFAULT_YELLOW_LINE);
  let right_lines = lines_on_right(lines, &yellow_line);
  let white_line = get_average_line(&right_lines, White).unwrap_or(DEFAULT_WHITE_LINE);
  let lane = get_midline(&yellow_line, &white_line);

  Lane::new(lane, yellow_line, white_line)
}

#[cfg(test)]
mod tests {
  use crate::detect_lane::lies_on_right;
  use cv::line::Colour::Red;
  use cv::line::{Line, LineSegment, Point, Vector};

  #[test]
  fn test_lies_on_right() {
    let boundary = Line::new(Point::new(1.0, 1.0), Vector::new(1.0, -1.0));
    assert!(lies_on_right(Point::new(2.0, 2.0), &boundary));
    assert!(!lies_on_right(Point::new(0.0, 0.0), &boundary));
    assert!(!lies_on_right(Point::new(1.0, 1.0), &boundary));
  }

  #[test]
  fn test_lines_on_right() {
    let lines: Vec<LineSegment> = vec![
      LineSegment::new(Red, Point::new(0.0, 0.0), Point::new(3.0, 3.0)),
      LineSegment::new(Red, Point::new(0.0, 0.0), Point::new(2.0, 2.0)),
      LineSegment::new(Red, Point::new(0.0, 0.0), Point::new(1.0, 1.0)),
    ];
    let boundary = Line::new(Point::new(1.0, 1.0), Vector::new(1.0, -1.0));
    let right_lines = super::lines_on_right(&lines, &boundary);
    assert_eq!(right_lines.len(), 1);
    assert!(right_lines.contains(&lines[0]));
  }
}
