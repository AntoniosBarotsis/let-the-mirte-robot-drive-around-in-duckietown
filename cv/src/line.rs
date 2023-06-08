use crate::image::CROP_HEIGHT;
use derive_more::{Add, Div, Mul, Neg, Sub, Sum};
use float_cmp::approx_eq;

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_YELLOW: &[[u8; 3]; 2] = &[[20, 100, 115], [45, 255, 255]];
pub static HSV_WHITE: &[[u8; 3]; 2] = &[[0, 0, 190], [179, 40, 255]];
pub static HSV_RED: &[[u8; 3]; 2] = &[[160, 100, 100], [20, 255, 255]];

/// given a colour type it will return the lower and upper bound of the range of that colour in HSV
///
/// # Panics
///
/// * `colour` - The colour of which the colour range needs to be extracted
///
/// Return an 2d-array with the lower bound on index 0 and upper bound on index 1
///
/// # Examples
///
/// ```
/// use cv::line::{get_colour, Colour};
///
/// let yellow = *get_colour(Colour::Yellow);
/// let white = *get_colour(Colour::White);
/// assert_ne!(white, yellow);
/// ```
pub fn get_colour(colour: Colour) -> &'static [[u8; 3]; 2] {
  match colour {
    Colour::White => HSV_WHITE,
    Colour::Yellow => HSV_YELLOW,
    Colour::Red => HSV_RED,
    _ => panic!("No HSV constants defined for {colour:?}!"),
  }
}
// Represents a line
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineSegment {
  pub colour: Colour,
  pub start: Point,
  pub end: Point,
}

impl LineSegment {
  pub fn new(colour: Colour, start: Point, end: Point) -> Self {
    Self { colour, start, end }
  }

  pub fn from_line(line: Line, colour: Colour) -> Self {
    let upper_y = Line::new(Point::new(0.0, CROP_HEIGHT), Vector::new(1.0, 0.0));
    let lower_y = Line::new(Point::new(0.0, 1.0), Vector::new(1.0, 0.0));

    let upper_y_intersection = line
      .intersect(&upper_y)
      .unwrap_or(Point::new(1.0, line.origin.y));
    let lower_y_intersection = line
      .intersect(&lower_y)
      .unwrap_or(Point::new(0.0, line.origin.y));

    Self::new(colour, lower_y_intersection, upper_y_intersection)
  }

  pub fn direction(&self) -> Vector {
    if self.start.y < self.end.y {
      Vector::from_point(self.start - self.end)
    } else {
      Vector::from_point(self.end - self.start)
    }
  }

  pub fn midpoint(&self) -> Point {
    (self.start + self.end) / 2.0
  }
}

// Represents a line color
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Colour {
  Red,
  Orange,
  Yellow,
  Green,
  Blue,
  Purple,
  Black,
  White,
}

// Represents a end coordinate of a line
#[derive(Debug, Clone, Copy, PartialEq, Add, Sub, Div, Mul, Sum)]
pub struct Point {
  pub x: f32,
  pub y: f32,
}

impl Point {
  pub fn new(x: f32, y: f32) -> Self {
    Self { x, y }
  }

  pub fn from_vector(vec: Vector) -> Self {
    Self::new(vec.x, vec.y)
  }

  pub const ORIGIN: Point = Point { x: 0.0, y: 0.0 };
}

#[derive(Debug, Clone, Copy, Add, Sub, Div, Mul, Sum, Neg)]
pub struct Vector {
  pub x: f32,
  pub y: f32,
}

impl Vector {
  pub fn new(x: f32, y: f32) -> Self {
    Self { x, y }
  }

  fn from_point(point: Point) -> Self {
    Self::new(point.x, point.y)
  }

  pub fn squared_length(&self) -> f32 {
    self.x * self.x + self.y * self.y
  }

  pub fn length(&self) -> f32 {
    f32::sqrt(self.squared_length())
  }
}

#[derive(Debug, Clone, Copy)]
pub struct Line {
  pub origin: Point,
  pub dir: Vector,
}

impl Line {
  pub fn new(origin: Point, dir: Vector) -> Self {
    Self { origin, dir }
  }

  pub fn from_line(line: &LineSegment) -> Self {
    Self {
      origin: line.start,
      dir: line.direction(),
    }
  }

  pub fn from_dir(dir: Vector) -> Self {
    Self {
      origin: Point::ORIGIN,
      dir,
    }
  }

  pub fn slope(&self) -> f32 {
    self.dir.y / self.dir.x
  }

  pub fn clamped_slope(&self) -> f32 {
    let slope = self.slope();
    if slope.is_nan() || slope.is_infinite() || slope > 100_000_000_000.0 {
      100_000_000_000.0
    } else {
      slope
    }
  }

  /// Returns the y value at the given x value on the vector
  fn y(&self, x: f32) -> f32 {
    self.slope() * (x - self.origin.x) + self.origin.y
  }

  pub fn intersect(&self, other: &Line) -> Option<Point> {
    // We clamp the slopes to still get a valid output even if the slopes are infinite or NaN
    let slope1 = self.clamped_slope();
    let slope2 = other.clamped_slope();
    // If slopes are the same, x will be invalid because the lines never intersect.
    if approx_eq!(f32, slope1, slope2, ulps = 2) {
      return None;
    }

    let x = (self.origin.x * slope1 - self.origin.y - other.origin.x * slope2 + other.origin.y)
      / (slope1 - slope2);
    let y = if self.slope().is_infinite() {
      other.y(x)
    } else {
      self.y(x)
    };

    Some(Point { x, y })
  }
}

#[derive(Debug, Clone, Copy)]
pub struct Lane {
  pub centre: Line,
  pub left: Line,
  pub right: Line,
}

impl Lane {
  pub fn new(lane: Line, left_line: Line, right_line: Line) -> Self {
    Self {
      centre: lane,
      left: left_line,
      right: right_line,
    }
  }

  pub fn get_coloured_segments(
    &self,
    lane_colour: Colour,
    left_colour: Colour,
    right_colour: Colour,
  ) -> Vec<LineSegment> {
    [
      (self.centre, lane_colour),
      (self.left, left_colour),
      (self.right, right_colour),
    ]
    .iter()
    .map(|&(line, color)| LineSegment::from_line(line, color))
    .collect()
  }
}

#[cfg(test)]
mod tests {
  use crate::line::{Colour, Line, LineSegment, Point, Vector};
  use float_cmp::assert_approx_eq;

  fn assert_float_eq(f1: f32, f2: f32) {
    assert_approx_eq!(f32, f1, f2, ulps = 0);
  }

  fn assert_point_eq(point1: Point, point2: Point) {
    assert_float_eq(point1.x, point2.x);
    assert_float_eq(point1.y, point2.y);
  }

  fn assert_vec_eq(vec1: Vector, vec2: Vector) {
    assert_float_eq(vec1.x, vec2.x);
    assert_float_eq(vec1.y, vec2.y);
  }

  fn assert_line_segment_eq(segment1: LineSegment, segment2: LineSegment) {
    assert_eq!(segment1.colour, segment2.colour);
    assert_point_eq(segment1.start, segment2.start);
    assert_point_eq(segment1.end, segment2.end);
  }

  fn assert_line_representation_eq(line1: Line, line2: Line) {
    assert_point_eq(line1.origin, line2.origin);
    assert_vec_eq(line1.dir, line2.dir);
  }

  #[test]
  fn create_point() {
    let point = Point::new(10.0, -5.0);
    assert_float_eq(point.x, 10.0);
    assert_float_eq(point.y, -5.0);
  }

  #[test]
  fn create_line() {
    let line = Line::new(Point::new(6.5, 3.0), Vector::new(1.0, 1.0));
    assert_point_eq(line.origin, Point::new(6.5, 3.0));
    assert_vec_eq(line.dir, Vector::new(1.0, 1.0));
  }

  #[test]
  fn line_inf_slope() {
    let line = Line::new(Point::new(5.0, 0.0), Vector::new(0.0, 10.0));
    let slope = line.slope();
    assert_float_eq(slope, f32::INFINITY);
  }

  #[test]
  fn line_slope() {
    let line = Line::new(Point::new(5.0, 0.0), Vector::new(1.0, 10.0));
    let slope = line.slope();
    assert_float_eq(slope, 10.0);
  }

  #[test]
  fn intersect() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector::new(1.0, 1.0));
    let line2 = Line::new(Point::new(0.0, 0.0), Vector::new(1.0, -1.0));
    let intersection = line1.intersect(&line2).expect("No intersection found!");
    assert_point_eq(intersection, Point::new(2.0, -2.0));
  }

  #[test]
  fn intersect_with_vertical() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector::new(0.0, 1.0));
    let line2 = Line::new(Point::new(0.0, 0.0), Vector::new(1.0, 1.0));
    let intersection = line1.intersect(&line2).expect("No intersection found!");
    assert_point_eq(intersection, Point::new(4.0, 4.0));
  }

  #[test]
  fn intersect_with_identical() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector::new(1.0, 1.0));
    let line2 = Line::new(Point::new(4.0, 0.0), Vector::new(1.0, 1.0));
    let intersection = line1.intersect(&line2);
    assert!(intersection.is_none());
  }

  #[test]
  fn intersect_with_same_slope() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector::new(1.0, 1.0));
    let line2 = Line::new(Point::new(8.0, 0.0), Vector::new(1.0, 1.0));
    let intersection = line1.intersect(&line2);
    assert!(intersection.is_none());
  }

  #[test]
  fn create_point_from_dir() {
    let dir = Vector::new(10.0, -5.0);
    let point = Point::from_vector(dir);
    assert_point_eq(point, Point::new(10.0, -5.0));
  }

  #[test]
  fn create_dir() {
    let dir = Vector::new(10.0, -5.0);
    assert_vec_eq(dir, Vector::new(10.0, -5.0));
  }

  #[test]
  fn create_dir_from_point() {
    let point = Point::new(10.0, -5.0);
    let dir = Vector::from_point(point);
    assert_vec_eq(dir, Vector::new(10.0, -5.0));
  }

  #[test]
  fn compute_squared_length() {
    let dir = Vector::new(10.0, -5.0);
    assert_float_eq(dir.squared_length(), 125.0);
  }

  #[test]
  fn compute_length() {
    let dir = Vector::new(3.0, -4.0);
    assert_float_eq(dir.length(), 5.0);
  }

  #[test]
  fn create_line_segment() {
    let segment = LineSegment::new(Colour::Red, Point::new(10.0, -5.0), Point::new(20.0, -10.0));
    assert_line_segment_eq(
      segment,
      LineSegment::new(Colour::Red, Point::new(10.0, -5.0), Point::new(20.0, -10.0)),
    );
  }

  #[test]
  fn create_line_segment_from_line() {
    let line = Line::new(Point::new(0.5, 1.0), Vector::new(0.0, -1.0));
    let segment = LineSegment::from_line(line, Colour::Red);
    assert_point_eq(segment.start, Point::new(0.5, 1.0));
  }

  #[test]
  fn create_line_segment_from_horizontal_line() {
    let line = Line::new(Point::new(0.5, 0.75), Vector::new(1.0, 0.0));
    let segment = LineSegment::from_line(line, Colour::Red);
    assert_point_eq(segment.start, Point::new(0.0, 0.75));
    assert_point_eq(segment.end, Point::new(1.0, 0.75));
  }

  #[test]
  fn get_direction() {
    let segment = LineSegment::new(Colour::Red, Point::new(10.0, -5.0), Point::new(20.0, -10.0));
    let dir = segment.direction();
    assert_vec_eq(dir, Vector::new(10.0, -5.0));
  }

  #[test]
  fn get_direction_from_inverted_coords() {
    let segment = LineSegment::new(Colour::Red, Point::new(10.0, 5.0), Point::new(20.0, 10.0));
    let dir = segment.direction();
    assert_vec_eq(dir, Vector::new(-10.0, -5.0));
  }

  #[test]
  fn get_midpoint() {
    let segment = LineSegment::new(Colour::Red, Point::new(10.0, -5.0), Point::new(20.0, -10.0));
    let midpoint = segment.midpoint();
    assert_point_eq(midpoint, Point::new(15.0, -7.5));
  }

  #[test]
  fn create_line_from_vector() {
    let vec = Vector::new(10.0, -5.0);
    let line = Line::from_dir(vec);
    assert_line_representation_eq(
      line,
      Line::new(Point::new(0.0, 0.0), Vector::new(10.0, -5.0)),
    );
  }
}
