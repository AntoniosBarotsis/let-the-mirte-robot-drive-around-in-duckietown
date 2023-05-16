// Represents a line color
#[derive(Debug, Clone, Copy)]
pub enum Color {
  Yellow,
  White,
  Red
}

// Represents a end coordinate of a line
#[derive(Debug, Clone, Copy)]
pub struct Pos {
  x: f64,
  y: f64,
}

// Represents a line
#[derive(Debug, Clone, Copy)]
pub struct Line {
  color: Color,
  pos1: Pos,
  pos2: Pos,
}