use super::detection::{get_average_line, lies_on_right};
use common::{
  geometry_msgs::{Point, Vector3},
  structs::colour::ColourEnum,
};
use ros::mirte_msgs::{Line, LineSegment};

#[test]
fn test_average_line() {
  let lines = vec![];
  let line = get_average_line(lines, ColourEnum::Yellow).unwrap();
  //assert_eq!()
}

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
