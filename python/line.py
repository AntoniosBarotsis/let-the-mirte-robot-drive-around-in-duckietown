from enum import Enum

class Colour(Enum):
  RED = 0
  ORANGE = 1
  YELLOW = 2
  GREEN = 3
  BLUE = 4
  PURPLE = 5
  BLACK = 6
  WHITE = 7

  def __str__(self):
    return self.name

class Point:
  def __init__(self, x: float, y: float):
    self.x = x
    self.y = y

  def __str__(self):
    return "({}, {})".format(self.x, self.y)

class LineSegment:
  def __init__(self, colour: Colour, start: Point, end: Point):
    self.colour = colour
    self.start = start
    self.end = end

  def __str__(self):
    return "LineSegment(colour={}, start={}, end={})".format(self.colour, self.start, self.end)
  
  def from_message(message):
    return LineSegment(Colour(message.colour.type), Point(message.start.x, message.start.y), Point(message.end.x, message.end.y))
