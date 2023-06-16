pub mod colour;
pub mod lane;
pub mod line;
pub mod line_segment;
pub mod object;
pub mod obstacle;
pub mod obstacle_list;
pub mod point;
pub mod threshold;
pub mod vector;

#[cfg(test)]
mod tests {
  use crate::{
    geometry_msgs::{Point, Vector3},
    mirte_msgs::{Line, LineSegment},
  };

  use super::colour::ColourEnum;

  #[allow(clippy::cast_possible_truncation)]
  fn assert_float_eq(f1: f64, f2: f64) {
    assert!(((f1 - f2).abs() as f32) < f32::EPSILON || f1 == f2);
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
