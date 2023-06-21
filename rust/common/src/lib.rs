pub mod logger;
pub mod structs;

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
