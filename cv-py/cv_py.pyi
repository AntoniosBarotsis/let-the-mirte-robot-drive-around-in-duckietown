from typing import List
from enum import Enum

class colour(Enum):
  red = 1
  orange = 2
  yellow = 3
  green = 4
  blue = 5
  purple = 6
  black = 7
  white = 8

class pos:
   x: float
   y: float

class line:
    colour: colour
    start: pos
    end: pos


def detect_line_type(colours: List[colour]) -> List[line]:
  """
  TODO: Copy paste the docs of the Rust method when they are added
  """

def detect_lane(colours: List[line]) -> List[line]:
  """
  TODO: Copy paste the docs of the Rust method when they are added
  """
