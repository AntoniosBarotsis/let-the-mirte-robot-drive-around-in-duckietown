// Represents a line color
#[derive(Debug, Clone, Copy)]
pub enum Colour {
  Yellow,
  White,
}

// Represents a end coordinate of a line
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct Pos {
  pub x: f32,
  pub y: f32,
}

// Represents a line
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct Line {
  pub colour: Colour,
  pub pos1: Pos,
  pub pos2: Pos,
}

// H: 0-179, S: 0-255, V: 0-255
pub static HSV_YELLOW: &[[u8; 3]; 2] = &[[25, 0, 0], [45, 255, 255]];
pub static HSV_WHITE: &[[u8; 3]; 2] = &[[0, 0, 150], [179, 60, 255]];
