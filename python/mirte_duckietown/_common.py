from enum import Enum
from dataclasses import dataclass


class Colour(Enum):
    """Enum for colours

    Represents the available colours for a line segment.
    """

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


@dataclass
class Point:
    """Point in 2D space

    A point with x and y coordinates.
    """

    x_coord: float
    y_coord: float

    def __str__(self):
        return f"({self.x_coord}, {self.y_coord})"


@dataclass
class Vector:
    """Vector in 2D space

    A vector with x and y components.
    """

    x_coord: float
    y_coord: float

    def __str__(self):
        return f"({self.x_coord}, {self.y_coord})"


@dataclass
class LineSegment:
    """Line segment in 2D space

    A line segment with a colour, start and end point.
    """

    colour: Colour
    start: Point
    end: Point

    def __str__(self):
        return f"LineSegment(colour={self.colour}, start={self.start}, end={self.end})"

    @staticmethod
    def fromMessage(message):
        """Converts a LineSegment message to a LineSegment object

        Parameters:
            message (mirte_msgs.msg.LineSegment): The message to convert

        Returns:
            LineSegment: The converted object
        """
        return LineSegment(
            Colour(message.colour.type),
            Point(message.start.x, message.start.y),
            Point(message.end.x, message.end.y),
        )


@dataclass
class Line:
    """Line in 2D space

    A line with an origin and direction.
    """

    origin: Point
    direction: Vector

    def __str__(self):
        return f"Line(origin={self.origin}, direction={self.direction})"

    @staticmethod
    def fromMessage(message):
        """Converts a Line message to a Line object

        Parameters:
            message (mirte_msgs.msg.Line): The message to convert

        Returns:
            Line: The converted object
        """
        return Line(
            Point(message.origin.x, message.origin.y),
            Vector(message.direction.x, message.direction.y),
        )
