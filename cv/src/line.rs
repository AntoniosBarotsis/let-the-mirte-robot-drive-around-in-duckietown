// Represents a line color
#[derive(Debug, Clone, Copy)]
pub enum Color {
  Yellow,
  White
}

// Represents a end coordinate of a line
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct Pos {
  x: f64,
  y: f64,
}

// Represents a line
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct Line {
  color: Color,
  pos1: Pos,
  pos2: Pos,
}

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_YELLOW: &[[f64; 3]; 2] = &[[25.0, 140.0, 100.0], [45.0, 255.0, 255.0]];
pub static HSV_WHITE: &[[f64; 3]; 2] = &[[0.0, 0.0, 150.0], [179.0, 60.0, 255.0]];