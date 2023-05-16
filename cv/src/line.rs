enum Color {
  Yellow,
  White,
  Red
}

struct Pos {
  x: f64,
  y: f64,
}

struct Line {
  color: Color,
  pos1: Pos,
  pos2: Pos,
}