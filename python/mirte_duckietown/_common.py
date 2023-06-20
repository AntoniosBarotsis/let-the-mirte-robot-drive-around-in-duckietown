from enum import Enum
from dataclasses import dataclass
import math


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
    The class has a start value, which indicates where the line is at the bottom of the image.
        -1 is at the left of the image, 1 is at the right of the image.
    The class has an angle value, which indicates the angle of the line in degrees.
        0 is pointing up, 90 is pointing right, -90 is pointing left.
    """

    origin: Point
    direction: Vector
    start: float
    angle: float

    def __str__(self):
        return f"Line(origin={self.origin}, direction={self.direction}, start={self.start}, angle={self.angle})"

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
            convert_angle_to_degrees(calculate_radians(message.direction.x, message.direction.y)),
            calculate_y1_intercept(
                message.origin.x,
                message.origin.y,
                message.origin.x + message.direction.x,
                message.origin.y + message.direction.y
            )
        )


@dataclass
class Lane:
    """Lane datastructure

    A datastructure containing three lines, representing the left, centre and right lines of a lane.
    """
    left_line: Line
    centre_line: Line
    right_line: Line

    @staticmethod
    def fromMessage(message):
        """Converts a Lane message to a Lane object

        Parameters:
            message (mirte_msgs.msg.Lane): The message to convert

        Returns:
            Lane: The converted object
        """
        return Lane(
            Line.fromMessage(message.left_line),
            Line.fromMessage(message.centre_line),
            Line.fromMessage(message.right_line)
        )

    def __str__(self):
        return f"Lane(left_line={self.left_line}, centre_line={self.centre_line}, right_line={self.right_line})"

# Some helper functions:


# Calculates an angle from a vector [x, y]
def calculate_radians(x, y):
    return math.atan2(y, x)


# Converts an angle in radians to degrees, where 0 degrees is straight up, and positive angles are clockwise
def convert_angle_to_degrees(angle):
    return math.degrees(angle) + 90


# Calculates the intercept of a line given by two points with the line y=1 (the bottom of the image)
def calculate_y1_intercept(x1, y1, x2, y2):
    if x1 == x2:
        return x1  # vertical line, so intercept will be the x coordinate
    a = (y2 - y1)/(x2 - x1)
    b = y1-a*x1
    return (1 - b)/a