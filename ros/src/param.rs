use cv::line::{Colour, Threshold};
use std::collections::HashMap;

/// Gets the threshold per colour from private parameters like `/thresholds/white` or
/// `/thresholds/yellow`. If it can't find the parameter, it uses the default threshold for that
/// colour.
pub fn get_thresholds() -> HashMap<Colour, Threshold> {
  [Colour::White, Colour::Yellow, Colour::Red]
    .iter()
    .map(|colour| {
      // It should always be possible to resolve the parameter name, so this will never panic on
      // the `expect` call.
      #[allow(clippy::expect_used)]
      let param =
        rosrust::param(&format!("~thresholds/{}", colour.parameter_name())).expect(&format!(
          "Unable to resolve parameter name '~thresholds/{}'",
          colour.parameter_name()
        ));
      let threshold = param
        .get()
        .unwrap_or_else(|_| Threshold::by_colour(*colour));
      let _ = param.set(&threshold);
      (*colour, threshold)
    })
    .collect()
}
