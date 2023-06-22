pub mod logger;
pub mod structs;
#[cfg(test)]
pub mod test;

rosrust::rosmsg_include!(
  geometry_msgs / Point,
  geometry_msgs / Vector3,
  mirte_msgs / Colour,
  mirte_msgs / Line,
  mirte_msgs / Lane,
  mirte_msgs / LineSegment,
  mirte_msgs / LineSegmentList,
  mirte_msgs / Object,
  mirte_msgs / Obstacle,
  mirte_msgs / ObstacleList,
);

fn float_eq(f1: f64, f2: f64) -> bool {
  (f1 - f2).abs() < f64::EPSILON || f1 == f2
}
