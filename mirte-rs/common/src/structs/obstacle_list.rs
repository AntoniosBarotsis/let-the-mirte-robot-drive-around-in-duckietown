use crate::mirte_duckietown_msgs::{Obstacle, ObstacleList};

impl From<Vec<Obstacle>> for ObstacleList {
  fn from(value: Vec<Obstacle>) -> Self {
    Self { obstacles: value }
  }
}
