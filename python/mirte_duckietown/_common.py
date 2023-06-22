import os
from datetime import datetime
from enum import Enum
from dataclasses import dataclass
import math
import yaml
from .sign import Sign


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
        return f"Line(origin={self.origin}, " \
               f"direction={self.direction}, " \
               f"start={self.start}, " \
               f"angle={self.angle})"

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
            calculateY1Intercept(
                message.origin.x,
                message.origin.y,
                message.origin.x + message.direction.x,
                message.origin.y + message.direction.y
            ),
            convertAngleToDegrees(calculateRadians(message.direction.x, message.direction.y))
        )


@dataclass
class AprilTag:
    """AprilTag

    Represents an AprilTag with an ID and timestamp.
    """

    tag_id: int
    timestamp: datetime

    def __str__(self):
        return f"AprilTag(tag_id={self.tag_id}, timestamp={self.timestamp})"

    def __eq__(self, other):
        return self.tag_id == other.tag_id

    def hasExpired(self, tag_life):
        """Checks if the AprilTag has expired

        Parameters:
            tag_life (int): The shelf life of the AprilTag in milliseconds

        Returns:
            bool: True if the AprilTag has expired, False otherwise
        """
        return (
            datetime.now() - self.timestamp
        ).total_seconds() * 1000 > tag_life

    def toSign(self):
        """Converts the AprilTag to a Sign

        Returns:
            Sign: The converted Sign
        """
        tag_db = TagDatabase()
        tag: dict = tag_db.lookup(self.tag_id)
        print(tag, self.tag_id)
        # Check if tag exists
        if tag is None:
            return None
        # If tag is a traffic sign, return the corresponding type
        if tag.get("tag_type") == "TrafficSign":
            return Sign(tag.get("traffic_sign_type"))
        # If tag is a street sign, return the street sign type
        if tag.get("tag_type") == "StreetName":
            return Sign("street")

        return None

    def getStreetName(self):
        """Gets the street name of the AprilTag

        Returns:
            str: The street name if the AprilTag is a street sign, None otherwise
        """
        tag_db = TagDatabase()
        tag: dict = tag_db.lookup(self.tag_id)
        if tag is None:
            return None
        # If tag is a street sign, return the street name
        street_name: str = tag.get("street_name")
        if street_name is None:
            return None
        # Remove trailing dots
        return street_name.rstrip(".")


class TagDatabase:
    """Database for AprilTags

    Reads the AprilTag database from a YAML file and provides a lookup function.
    """

    _instance = None
    _initialized = False

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        # Don't initialize twice
        if self._initialized:
            return
        # Load the database
        file_path = os.path.join(os.path.dirname(__file__), "apriltagsDB.yaml")
        with open(file_path, encoding="utf8") as file:
            self.data = yaml.load(file, Loader=yaml.FullLoader)
        # Mark as initialized
        self._initialized = True

    def lookup(self, tag_id):
        """Looks up the AprilTag in the database

        Parameters:
            tag_id (int): The ID of the AprilTag

        Returns:
            dict: The AprilTag if found, None otherwise
        """
        for item in dict(self.data)['standalone_tags']:
            if item["id"] == tag_id:
                return item
        return None


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
            Line.fromMessage(message.left),
            Line.fromMessage(message.centre),
            Line.fromMessage(message.right)
        )

    def __str__(self):
        return f"Lane(left_line={self.left_line}, " \
               f"centre_line={self.centre_line}, " \
               f"right_line={self.right_line})"

# Some helper functions:
# pylint: disable=invalid-name
# pylint: disable=missing-function-docstring

# Calculates an angle from a vector [x, y]
def calculateRadians(x, y):
    return math.atan2(y, x)


# Converts an angle in radians to degrees
# 0 degrees is straight up, and positive angles are clockwise
def convertAngleToDegrees(angle):
    return math.degrees(angle) + 90


# Calculates the intercept of a line given by two points with the line y=1 (the bottom of the image)
def calculateY1Intercept(x1, y1, x2, y2):
    if x1 == x2:
        return x1  # vertical line, so intercept will be the x coordinate
    a = (y2 - y1)/(x2 - x1)
    if a == 0:
        return math.inf  # horizontal line, so intercept will be infinity
    b = y1-a*x1
    return (1 - b)/a
