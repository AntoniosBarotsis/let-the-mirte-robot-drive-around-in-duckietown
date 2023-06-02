from typing import List
from enum import Enum

class Lane:
   centre: Line
   left: Line
   right: Line

class LineSegment:
   """Represents a line."""
   colour: Colour
   start: Point
   end: Point

class Line:
   origin: Point
   direction: Vector

class Vector:
   x: float
   y: float

class Colour(Enum):
  """Represents a line color."""
  red = 1
  orange = 2
  yellow = 3
  green = 4
  blue = 5
  purple = 6
  black = 7
  white = 8

class Point:
   """Represents a end coordinate of a line."""
   x: float
   y: float

def detect_line_type(colours: List[Colour]) -> List[LineSegment]:
  """
  Given a image and a vector of colours this method will detect lines in the image for all given colours.
  
  * `colours` - A vector of all the colours if which you want to detect the lines

  Returns a result with a vector of all the lines found in the image

  """

def detect_lane(colours: List[LineSegment]) -> Lane:
  """
  Detects the lane based on given line segments. Returns a `Lane` if successful.
  """
