import datetime
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

class AprilTag:
    def __init__(self, id: int, timestamp: datetime.datetime) -> None:
        self.id = id
        self.timestamp = timestamp
    
    def __eq__(self, __value: object) -> bool:
        return self.id == __value.id

class Sign(Enum):
    CROSS_INTERSECTION = 1
    YIELD = 2
    NO_TURNING_RIGHT = 3
    NO_TURNING_LEFT = 4
    INTERSECTION_RIGHT = 5
    INTERSECTION_LEFT = 6
    T_INTERSECTION = 7
    TRAFFIC_LIGHT = 8
    PEDESTRIAN_CROSSING = 9
    PARKING = 10
    BARFOOT_ST = 11
    DUDEK_ST = 12
    FORBES_AVE = 13
    PINEAU_AVE = 14
    KELLY_ST = 15
    URTASUN_RD = 16
    WASLANDER_ST = 17
    SHARF_ST = 18
    SHOELLIG_ST = 19
    STOP = 20
    UNDEFINED = 21

def id_to_sign(tag_id: int) -> Sign:
    if 13 <= tag_id <= 16:
        return Sign.CROSS_INTERSECTION
    elif 20 <= tag_id <= 28:
        return Sign.STOP
    elif tag_id == 39:
        return Sign.YIELD
    elif tag_id == 40:
        return Sign.NO_TURNING_RIGHT
    elif tag_id == 41:
        return Sign.NO_TURNING_LEFT
    elif 57 <= tag_id <= 58:
        return Sign.INTERSECTION_RIGHT
    elif 61 <= tag_id <= 62:
        return Sign.INTERSECTION_LEFT
    elif 65 <= tag_id <= 67:
        return Sign.T_INTERSECTION
    elif 74 <= tag_id <= 77:
        return Sign.TRAFFIC_LIGHT
    elif 95 <= tag_id <= 102:
        return Sign.PEDESTRIAN_CROSSING
    elif tag_id == 125:
        return Sign.PARKING
    elif tag_id == 530:
        return Sign.BARFOOT_ST
    elif tag_id == 531:
        return Sign.DUDEK_ST
    elif tag_id == 532:
        return Sign.FORBES_AVE
    elif tag_id == 533:
        return Sign.PINEAU_AVE
    elif tag_id == 534:
        return Sign.KELLY_ST
    elif tag_id == 535:
        return Sign.URTASUN_RD
    elif tag_id == 537:
        return Sign.WASLANDER_ST
    elif tag_id == 541:
        return Sign.SHARF_ST
    elif tag_id == 542:
        return Sign.SHOELLIG_ST
    else:
        return Sign.UNDEFINED
