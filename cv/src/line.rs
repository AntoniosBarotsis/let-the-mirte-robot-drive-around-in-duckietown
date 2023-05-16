pub enum Color {
  Yellow,
  White,
  Red
}

pub struct Pos {
  x: f64,
  y: f64,
}

pub struct Line {
  color: Color,
  pos1: Pos,
  pos2: Pos,
}