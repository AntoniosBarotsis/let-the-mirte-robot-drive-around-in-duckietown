from enum import Enum


class Object(Enum):
    """Enum for objects

    Represents the available objects for an obstacle.
    """

    MIRTE = 0
    DUCK = 1

    def __str__(self):
        return self.name
