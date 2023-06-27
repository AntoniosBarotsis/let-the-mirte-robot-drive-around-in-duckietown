import math

# Some parameter names are intentionally shorty to be easier to read. Therefore,
# we disable the invalid name warning:
# pylint: disable=invalid-name


def calculateRadians(x, y):
    """Calculates an angle from a vector

    Parameters:
        x (float): The x component of the vector
        y (float): The y component of the vector

    Returns:
        float: The angle in radians
    """
    return math.atan2(y, x)


def convertAngleToDegrees(angle):
    """Converts an angle in radians to degrees and turns it 90 degrees

    Parameters:
        angle (float): The angle in radians

    Returns:
        float: The angle in degrees
    """
    return math.degrees(angle) + 90


def calculateY1Intercept(x1, y1, x2, y2):
    """Calculates the intercept of the given line with the line y=1 which
       represents the bottom of the image

    Parameters:
        x1 (float): The x coordinate of the first point
        y1 (float): The y coordinate of the first point
        x2 (float): The x coordinate of the second point
        y2 (float): The y coordinate of the second point

    Returns:
        float: The x coordinate of the intercept
    """
    # In case of a vertical line, the intercept is the x coordinate
    if x1 == x2:
        return x1
    a = (y2 - y1) / (x2 - x1)
    # In case of a horizontal line, the intercept is infinite
    if a == 0:
        return math.inf
    b = y1 - a * x1
    return (1 - b) / a


def intersectWithHorizontalLine(line, height):
    """Calculates the intersection of two lines

    Parameters:
        line1 (Line): The first line
        height (float): The height of the horizontal line

    Returns:
        float: The x value of the instersection
    """
    # In case of a horizontal line, there is no intersection
    if line.direction.y_coord == 0:
        return None
    return (height - line.start) / line.direction.y_coord
