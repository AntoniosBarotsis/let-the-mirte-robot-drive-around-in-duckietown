use opencv::{
  highgui::{imshow, wait_key},
  prelude::*,
};

/// Showcases constructing a [Mat] (which we need for any `OpenCV` processing) from vectors and
/// arrays.
///
///
/// # Notes
///
/// It is worth noting that even though arrays are cool and all, I was getting stack overflows
/// with matrices of much smaller sizes than what the mirte camera supports. Another argument
/// against arrays is that we would need to hardcode the camera resolution which goes against
/// the idea of modularity so we might want to avoid them.
fn main() {
  // Using vectors
  let input = (0..250)
    .map(|el| match el {
      ..=124 => (0..250).map(|_el| 0.0).collect::<Vec<_>>(),
      125..=250 => (0..250).map(|_el| 1.0).collect::<Vec<_>>(),
      _ => unreachable!(),
    })
    .collect::<Vec<_>>();

  display(&input);

  // Using arrays
  let input: [[f32; 250]; 250] = std::array::from_fn(|a| match a {
    ..=124 => [1.0; 250],
    125..=250 => [0.0; 250],
    _ => unreachable!(),
  });

  display(&input);
}

fn display(input: &[impl AsRef<[f32]>]) {
  let mat = Mat::from_slice_2d(input).expect("mat creation");
  imshow("test", &mat).expect("open window");
  let _res = wait_key(0).expect("keep window open");
}
