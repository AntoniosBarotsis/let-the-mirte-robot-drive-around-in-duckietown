use crate::{
  geometry_msgs::Point,
  mirte_msgs::{Object, Obstacle},
};

impl Obstacle {
  pub fn new(location: Point, diameter: f32, object: Object) -> Self {
    Self {
      location,
      diameter,
      object,
    }
  }
}
