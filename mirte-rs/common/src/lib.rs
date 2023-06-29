pub mod logger;
pub mod structs;

rosrust::rosmsg_include!(
  geometry_msgs / Point,
  geometry_msgs / Vector3,
  mirte_duckietown_msgs / Colour,
  mirte_duckietown_msgs / Line,
  mirte_duckietown_msgs / Lane,
  mirte_duckietown_msgs / LineSegment,
  mirte_duckietown_msgs / LineSegmentList,
  mirte_duckietown_msgs / Object,
  mirte_duckietown_msgs / Obstacle,
  mirte_duckietown_msgs / ObstacleList,
);

pub const IMAGE_CROP_HEIGHT: f32 = 0.58;

#[allow(clippy::cast_possible_truncation)]
fn float_eq(f1: f64, f2: f64) -> bool {
  ((f1 - f2).abs() as f32) < f32::EPSILON || f1 == f2
}
