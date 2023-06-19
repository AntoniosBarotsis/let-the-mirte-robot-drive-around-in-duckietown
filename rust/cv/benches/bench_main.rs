use criterion::criterion_main;

mod benchmarks;

criterion_main! {
  benchmarks::detect_lines::detect_lines_bench,
}
