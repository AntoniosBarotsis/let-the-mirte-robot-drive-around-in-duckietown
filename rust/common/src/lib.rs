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
