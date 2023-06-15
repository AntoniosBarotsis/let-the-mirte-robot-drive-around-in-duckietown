use common::mirte_msgs::Colour;
use criterion::{criterion_group, Criterion};
use cv::detect_lines::detect_line_type;
use opencv::imgcodecs::{imread, IMREAD_GRAYSCALE};
use std::collections::HashMap;

fn criterion_benchmark(c: &mut Criterion) {
  #[allow(clippy::expect_used)]
  {
    let img = imread("../assets/input_real.jpg", IMREAD_GRAYSCALE).expect("open image");

    let _res = c.bench_function("detect lines (30 fps)", |b| {
      let input = (0..30).map(|_| img.clone()).collect::<Vec<_>>();

      b.iter(|| {
        for img in input.clone() {
          let colours = vec![
            Colour {
              type_: Colour::YELLOW,
            },
            Colour {
              type_: Colour::WHITE,
            },
          ];
          let _lines = detect_line_type(&img, &HashMap::new(), colours)
            .expect("Line detection does not crash.");
        }
      });
    });

    let _res = c.bench_function("detect lines (single)", |b| {
      b.iter(|| {
        let colours = vec![
          Colour {
            type_: Colour::YELLOW,
          },
          Colour {
            type_: Colour::WHITE,
          },
        ];
        let _lines = detect_line_type(&img.clone(), &HashMap::new(), colours)
          .expect("Line detection does not crash.");
      });
    });
  }
}

criterion_group!(detect_lines_bench, criterion_benchmark);
