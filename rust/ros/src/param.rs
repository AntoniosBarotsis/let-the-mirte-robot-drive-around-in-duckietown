use crate::init;
use common::mirte_msgs::Colour;
use cv::line::Threshold;
use std::collections::HashMap;

/// Gets the threshold per colour from private parameters like `/thresholds/white` or
/// `/thresholds/yellow`. If it can't find the parameter, it uses the default threshold for that
/// colour.
pub fn get_thresholds() -> HashMap<Colour, Threshold> {
  init();
  [
    Colour {
      type_: Colour::WHITE,
    },
    Colour {
      type_: Colour::YELLOW,
    },
    Colour { type_: Colour::RED },
  ]
  .iter()
  .map(|&colour| {
    let threshold = get_threshold(colour).unwrap_or_else(|| Threshold::by_colour(colour));
    (colour, threshold)
  })
  .collect()
}

fn get_threshold(colour: Colour) -> Option<Threshold> {
  let parameter_name = format!("~thresholds/{}", colour.parameter_name());
  let param = rosrust::param(&parameter_name)?;
  let threshold = param.get().ok()?;
  param.set(&threshold).unwrap_or_else(|_| {
    rosrust::ros_warn!("Unable to set parameter '{parameter_name}'");
  });
  threshold
}
