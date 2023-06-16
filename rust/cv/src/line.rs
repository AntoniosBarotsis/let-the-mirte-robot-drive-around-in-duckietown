use common::structs::colour::ColourEnum;
use serde::{Deserialize, Serialize};

// H: 0-179, S: 0-255, V: 0-255
pub type ColourValue = [u8; 3];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
pub struct Threshold {
  pub lower: ColourValue,
  pub upper: ColourValue,
}

impl Threshold {
  pub fn from(lower: ColourValue, upper: ColourValue) -> Self {
    Self { lower, upper }
  }

  /// given a colour type, it will return the default lower and upper bound of the range of that colour in HSV
  ///
  /// # Panics
  ///
  /// When called with a colour other than `White`, `Yellow` or `Red`.
  ///
  /// * `colour` - The colour of which the colour range needs to be extracted
  ///
  /// Return an 2d-array with the lower bound on index 0 and upper bound on index 1
  ///
  /// # Examples
  ///
  /// ```
  /// use cv::line::Threshold;
  /// use common::structs::colour::ColourEnum;
  ///
  /// let yellow = Threshold::by_colour(ColourEnum::Yellow);
  /// let white = Threshold::by_colour(ColourEnum::White);
  /// assert_ne!(white, yellow);
  /// ```
  pub fn by_colour(colour: ColourEnum) -> Self {
    match colour {
      ColourEnum::White => Self {
        lower: [0, 0, 190],
        upper: [179, 40, 255],
      },
      ColourEnum::Yellow => Self {
        lower: [20, 100, 115],
        upper: [45, 255, 255],
      },
      ColourEnum::Red => Self {
        lower: [165, 90, 60],
        upper: [10, 255, 255],
      },
      _ => panic!("No colour threshold defined for {colour:?}!"),
    }
  }
}

#[cfg(test)]
mod tests {
  use common::{
    geometry_msgs::{Point, Vector3},
    mirte_msgs::{Line, LineSegment},
    structs::colour::ColourEnum,
  };
  use float_cmp::assert_approx_eq;

  fn assert_float_eq(f1: f64, f2: f64) {
    assert_approx_eq!(f64, f1, f2, ulps = 0);
  }

  fn assert_point_eq(point1: Point, point2: Point) {
    assert_float_eq(point1.x, point2.x);
    assert_float_eq(point1.y, point2.y);
  }

  fn assert_vec_eq(vec1: Vector3, vec2: Vector3) {
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
    assert_vec_eq(line1.direction, line2.direction);
  }

  #[test]
  fn create_point() {
    let point = Point::new(10.0, -5.0);
    assert_float_eq(point.x, 10.0);
    assert_float_eq(point.y, -5.0);
  }

  #[test]
  fn create_line() {
    let line = Line::new(Point::new(6.5, 3.0), Vector3::new(1.0, 1.0));
    assert_point_eq(line.origin, Point::new(6.5, 3.0));
    assert_vec_eq(line.direction, Vector3::new(1.0, 1.0));
  }

  #[test]
  fn line_inf_slope() {
    let line = Line::new(Point::new(5.0, 0.0), Vector3::new(0.0, 10.0));
    let slope = line.slope();
    assert_float_eq(slope, f64::INFINITY);
  }

  #[test]
  fn line_slope() {
    let line = Line::new(Point::new(5.0, 0.0), Vector3::new(1.0, 10.0));
    let slope = line.slope();
    assert_float_eq(slope, 10.0);
  }

  #[test]
  #[allow(clippy::expect_used)]
  fn intersect() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector3::new(1.0, 1.0));
    let line2 = Line::new(Point::new(0.0, 0.0), Vector3::new(1.0, -1.0));
    let intersection = line1.intersect(&line2).expect("No intersection found!");
    assert_point_eq(intersection, Point::new(2.0, -2.0));
  }

  #[test]
  #[allow(clippy::expect_used)]
  fn intersect_with_vertical() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector3::new(0.0, 1.0));
    let line2 = Line::new(Point::new(0.0, 0.0), Vector3::new(1.0, 1.0));
    let intersection = line1.intersect(&line2).expect("No intersection found!");
    assert_point_eq(intersection, Point::new(4.0, 4.0));
  }

  #[test]
  fn intersect_with_identical() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector3::new(1.0, 1.0));
    let line2 = Line::new(Point::new(4.0, 0.0), Vector3::new(1.0, 1.0));
    let intersection = line1.intersect(&line2);
    assert!(intersection.is_none());
  }

  #[test]
  fn intersect_with_same_slope() {
    let line1 = Line::new(Point::new(4.0, 0.0), Vector3::new(1.0, 1.0));
    let line2 = Line::new(Point::new(8.0, 0.0), Vector3::new(1.0, 1.0));
    let intersection = line1.intersect(&line2);
    assert!(intersection.is_none());
  }

  #[test]
  fn create_point_from_dir() {
    let dir = Vector3::new(10.0, -5.0);
    let point = Point::from_vector(dir);
    assert_point_eq(point, Point::new(10.0, -5.0));
  }

  #[test]
  fn create_dir() {
    let dir = Vector3::new(10.0, -5.0);
    assert_vec_eq(dir, Vector3::new(10.0, -5.0));
  }

  #[test]
  fn create_dir_from_point() {
    let point = Point::new(10.0, -5.0);
    let dir = Vector3::from_point(point);
    assert_vec_eq(dir, Vector3::new(10.0, -5.0));
  }

  #[test]
  fn compute_squared_length() {
    let dir = Vector3::new(10.0, -5.0);
    assert_float_eq(dir.squared_length(), 125.0);
  }

  #[test]
  fn compute_length() {
    let dir = Vector3::new(3.0, -4.0);
    assert_float_eq(dir.length(), 5.0);
  }

  #[test]
  fn create_line_segment() {
    let segment = LineSegment::new(
      ColourEnum::Red,
      Point::new(10.0, -5.0),
      Point::new(20.0, -10.0),
    );
    assert_line_segment_eq(
      segment,
      LineSegment::new(
        ColourEnum::Red,
        Point::new(10.0, -5.0),
        Point::new(20.0, -10.0),
      ),
    );
  }

  #[test]
  fn create_line_segment_from_line() {
    let line = Line::new(Point::new(0.5, 1.0), Vector3::new(0.0, -1.0));
    let segment = LineSegment::from_line(line, ColourEnum::Red);
    assert_point_eq(segment.start, Point::new(0.5, 1.0));
  }

  #[test]
  fn create_line_segment_from_horizontal_line() {
    let line = Line::new(Point::new(0.5, 0.75), Vector3::new(1.0, 0.0));
    let segment = LineSegment::from_line(line, ColourEnum::Red);
    assert_point_eq(segment.start, Point::new(0.0, 0.75));
    assert_point_eq(segment.end, Point::new(1.0, 0.75));
  }

  #[test]
  fn get_direction() {
    let segment = LineSegment::new(
      ColourEnum::Red,
      Point::new(10.0, -5.0),
      Point::new(20.0, -10.0),
    );
    let dir = segment.direction();
    assert_vec_eq(dir, Vector3::new(10.0, -5.0));
  }

  #[test]
  fn get_direction_from_inverted_coords() {
    let segment = LineSegment::new(
      ColourEnum::Red,
      Point::new(10.0, 5.0),
      Point::new(20.0, 10.0),
    );
    let dir = segment.direction();
    assert_vec_eq(dir, Vector3::new(-10.0, -5.0));
  }

  #[test]
  fn get_midpoint() {
    let segment = LineSegment::new(
      ColourEnum::Red,
      Point::new(10.0, -5.0),
      Point::new(20.0, -10.0),
    );
    let midpoint = segment.midpoint();
    assert_point_eq(midpoint, Point::new(15.0, -7.5));
  }

  #[test]
  fn create_line_from_vector() {
    let vec = Vector3::new(10.0, -5.0);
    let line = Line::from_dir(vec);
    assert_line_representation_eq(
      line,
      Line::new(Point::new(0.0, 0.0), Vector3::new(10.0, -5.0)),
    );
  }
}
