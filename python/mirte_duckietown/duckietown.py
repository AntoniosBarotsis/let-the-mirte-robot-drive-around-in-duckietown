from ._topic import Subscriber

# The height of the stop line in the camera image.
STOP_LINE_THRESHOLD_HEIGHT = 0.4


class Camera:
    """Camera API

    This class allows you to get access to useful camera data.
    """

    def __init__(self, subscriber=None):
        """Initialises a new camera

        Parameters:
            subscriber (Subscriber): The subscriber to use for fetching ROS topics. If
                None, a new subscriber will be created.
        """
        if subscriber is None:
            self.__subscriber = Subscriber()
        else:
            self.__subscriber = subscriber

    def getLines(self):
        """Gets line segments from the camera

        Returns:
            list: List of LineSegment objects
        """
        return self.__subscriber.getLines()

    def getStopLine(self):
        """Gets the stop line from the camera

        Returns:
            Line: Stop line
        """
        return self.__subscriber.getStopLine()

    def getStopLineHeight(self):
        """Gets the height of the stop line in the camera image.

        Returns:
            float: The height (between 0.0 and 1.0) of the stop line in the
            camera image or None if the stop line is not visible.
        """
        stop_line = self.__subscriber.getStopLine()
        if stop_line is None or stop_line.direction.x_coord == 0:
            return None

        y_intercept = stop_line.origin.y_coord + (
            0.5 - stop_line.origin.x_coord
        ) * (stop_line.direction.y_coord / stop_line.direction.x_coord)

        return 1.0 - y_intercept

    def seesStopLine(self):
        """Checks if the robot sees the stop line close enough

        Returns:
            bool: True if the robot sees the stop line close enough, False otherwise
        """
        stop_line_height = self.getStopLineHeight()
        if stop_line_height is None:
            return False
        return stop_line_height < STOP_LINE_THRESHOLD_HEIGHT


def createCamera():
    """Creates a Camera object

    Returns:
        Camera: The created Camera object
    """
    return Camera()
