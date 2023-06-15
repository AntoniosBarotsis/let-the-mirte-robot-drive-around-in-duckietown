pub mod structs;

rosrust::rosmsg_include!(
  mirte_msgs / Point,
  mirte_msgs / Colour,
  mirte_msgs / Vector,
  mirte_msgs / Line,
  mirte_msgs / Lane,
  mirte_msgs / LineSegment,
  mirte_msgs / LineSegmentList,
  mirte_msgs / Object,
  mirte_msgs / Obstacle,
  mirte_msgs / ObstacleList,
);
