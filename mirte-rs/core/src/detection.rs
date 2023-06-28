use common::{
  geometry_msgs::{Point, Vector3},
  structs::colour::ColourEnum,
};
use ros::mirte_duckietown_msgs::{Colour, Lane};
use ros::mirte_duckietown_msgs::{Line, LineSegment};

// The minimum length of an average line for it to be significant
const MINIMUM_LENGTH: f64 = 0.15;
// The slope at which the direction of line should flip
const DIRECTION_SLOPE: f64 = 0.25;

// Default lines for when no lines are found
const DEFAULT_YELLOW_LINE: Line = Line {
  origin: Point::new(0.0, 1.0),
  direction: Vector3::new(0.25, -1.0),
};
const DEFAULT_WHITE_LINE: Line = Line {
  origin: Point::new(1.0, 1.0),
  direction: Vector3::new(-0.25, -1.0),
};

/// Averages all lines with a given colour weighted by their length
fn get_average_line(lines: &[LineSegment], colour: ColourEnum) -> Option<Line> {
  // Get lines with given colour
  let coloured_lines: Vec<LineSegment> = lines
    .iter()
    .filter(|line| line.colour == colour.into())
    .copied()
    .collect();

  // Get average line with line length as weight
  let weighted_dir: Vector3 = coloured_lines
    .iter()
    .map(|line| -> Vector3 {
      let sign: f64 = if line.colour.type_ == Colour::WHITE {
        -1.0
      } else {
        1.0
      };
      let threshold: Line = Line::from_dir(Vector3::new(sign, DIRECTION_SLOPE));
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
  let total_squared_length: f64 = coloured_lines
    .iter()
    .map(|line| line.direction().squared_length())
    .sum();
  let avg_pos: Point = coloured_lines.iter().fold(Point::ORIGIN, |pos, line| {
    pos + line.midpoint() * line.direction().squared_length()
  }) / total_squared_length;

  Some(Line {
    origin: avg_pos,
    direction: weighted_dir,
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
    let dir1 = line1.direction;
    let dir2 = line2.direction;
    let direction = dir1 * dir2.length() + dir2 * dir1.length();
    Line::new(intersection - Point::from_vector(direction), direction)
  })
}

/// Returns the midline between `line1` and `line2`, or a default midline if they don't intersect
fn get_midline(line1: &Line, line2: &Line) -> Line {
  bisect(line1, line2).unwrap_or({
    let midpoint = (line1.origin + line2.origin) / 2.0;
    Line::new(midpoint, line1.direction)
  })
}

/// Detects the lane based on given line segments.
pub fn detect_lane(lines: &[LineSegment]) -> Lane {
  let yellow_line = get_average_line(lines, ColourEnum::Yellow).unwrap_or(DEFAULT_YELLOW_LINE);
  let right_lines = lines_on_right(lines, &yellow_line);
  let white_line = get_average_line(&right_lines, ColourEnum::White).unwrap_or(DEFAULT_WHITE_LINE);
  let lane = get_midline(&yellow_line, &white_line);

  Lane::new(lane, yellow_line, white_line)
}

/// Detects the stop line based on given line segments. Returns a line with direction 0, 0 if no
/// stop line is found.
pub fn detect_stop_line(lines: &[LineSegment]) -> Line {
  get_average_line(lines, ColourEnum::Red)
    .unwrap_or(Line::new(Point::new(0.0, 0.0), Vector3::new(0.0, 0.0)))
}

#[cfg(test)]
mod tests {
  use common::{
    geometry_msgs::{Point, Vector3},
    structs::colour::ColourEnum,
  };
  use ros::mirte_duckietown_msgs::{Line, LineSegment};

  use crate::detection::lies_on_right;

  #[test]
  fn test_lies_on_right() {
    let boundary = Line::new(Point::new(1.0, 1.0), Vector3::new(1.0, -1.0));
    assert!(lies_on_right(Point::new(2.0, 2.0), &boundary));
    assert!(!lies_on_right(Point::new(0.0, 0.0), &boundary));
    assert!(!lies_on_right(Point::new(1.0, 1.0), &boundary));
  }

  #[test]
  fn test_lines_on_right() {
    let lines: Vec<LineSegment> = vec![
      LineSegment::new(ColourEnum::Red, Point::new(0.0, 0.0), Point::new(3.0, 3.0)),
      LineSegment::new(ColourEnum::Red, Point::new(0.0, 0.0), Point::new(2.0, 2.0)),
      LineSegment::new(ColourEnum::Red, Point::new(0.0, 0.0), Point::new(1.0, 1.0)),
    ];
    let boundary = Line::new(Point::new(1.0, 1.0), Vector3::new(1.0, -1.0));
    let right_lines = super::lines_on_right(&lines, &boundary);
    assert_eq!(right_lines.len(), 1);
    assert!(right_lines.contains(&lines[0]));
  }
}
