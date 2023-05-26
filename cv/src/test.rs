use opencv::{core::Size_, prelude::MatTraitConstManual};

use crate::{
  detect_lines::get_lines,
  image::{convert_to_gray, dbg_mat, downscale},
  line::Colour,
};

fn test() {
  let mat = dbg_mat("../assets/test_images/test_image_2.png").expect("couldn't get image");
  let mat_gray = convert_to_gray(&mat).expect("couldn't get gray image");
  let line_vec = get_lines(
    &mat_gray,
    Colour::Yellow,
    Size_ {
      width: 320,
      height: 240,
    },
    0.0,
  )
  .expect("couldn't detect a line");
  assert_eq!(line_vec.len(), 1);
  assert!(line_vec[0].start.x <= 170.0 && line_vec[0].start.x >= 150.0);
  assert!(line_vec[0].end.x <= 170.0 && line_vec[0].end.x >= 150.0);
}
