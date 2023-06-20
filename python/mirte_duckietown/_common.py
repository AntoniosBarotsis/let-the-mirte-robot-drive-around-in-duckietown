from datetime import datetime
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


class Sign(Enum):
    """Enum for signs

    Represents the available signs for an AprilTag.
    """

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

    def __str__(self):
        return self.name


@dataclass
class AprilTag:
    """AprilTag

    Represents an AprilTag with an ID and timestamp.
    """

    value: int
    timestamp: datetime

    _SIGN_MAPPING = {
        range(13, 17): Sign.CROSS_INTERSECTION,
        range(20, 29): Sign.STOP,
        39: Sign.YIELD,
        40: Sign.NO_TURNING_RIGHT,
        41: Sign.NO_TURNING_LEFT,
        range(57, 59): Sign.INTERSECTION_RIGHT,
        range(61, 63): Sign.INTERSECTION_LEFT,
        range(65, 68): Sign.T_INTERSECTION,
        range(74, 78): Sign.TRAFFIC_LIGHT,
        range(95, 103): Sign.PEDESTRIAN_CROSSING,
        125: Sign.PARKING,
        530: Sign.BARFOOT_ST,
        531: Sign.DUDEK_ST,
        532: Sign.FORBES_AVE,
        533: Sign.PINEAU_AVE,
        534: Sign.KELLY_ST,
        535: Sign.URTASUN_RD,
        537: Sign.WASLANDER_ST,
        541: Sign.SHARF_ST,
        542: Sign.SHOELLIG_ST,
    }

    def __str__(self):
        return f"AprilTag(value={self.value}, timestamp={self.timestamp})"

    def __eq__(self, other):
        return self.value == other.value

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
        for value_range, sign in self._SIGN_MAPPING.items():
            if isinstance(value_range, int) and self.value == value_range:
                return sign
            elif isinstance(value_range, range) and self.value in value_range:
                return sign
