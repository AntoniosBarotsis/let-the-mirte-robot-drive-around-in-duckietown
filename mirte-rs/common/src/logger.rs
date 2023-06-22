#[macro_export]
macro_rules! debug {
  ($($arg:tt)*) => {{
      #[cfg(debug_assertions)]
      println!($($arg)*);
  }};
}

#[macro_export]
macro_rules! edebug {
  ($($arg:tt)*) => {{
    #[cfg(debug_assertions)]
      eprintln!($($arg)*);
  }};
}
