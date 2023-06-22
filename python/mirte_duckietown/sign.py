from enum import Enum


class Sign(Enum):
    """Enum for signs

    Represents the available signs.
    """

    STOP = "stop"
    YIELD = "yield"
    NO_RIGHT_TURN = "no-right-turn"
    NO_LEFT_TURN = "no-left-turn"
    DO_NOT_ENTER = "do-not-enter"
    ONEWAY_RIGHT = "oneway-right"
    ONEWAY_LEFT = "oneway-left"
    FOUR_WAY_INTERSECT = "4-way-intersect"
    RIGHT_T_INTERSECT = "right-T-intersect"
    LEFT_T_INTERSECT = "left-T-intersect"
    T_INTERSECTION = "T-intersection"
    PEDESTRIAN = "pedestrian"
    T_LIGHT_AHEAD = "t-light-ahead"
    DUCK_CROSSING = "duck-crossing"
    PARKING = "parking"
    STREET = "street"

    def __str__(self):
        return self.value
