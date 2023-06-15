use crate::mirte_msgs::{Object, Obstacle, Point};

impl Obstacle {
  pub fn new(location: Point, diameter: f32, object: Object) -> Self {
    Self {
      location,
      diameter,
      object,
    }
  }
}
