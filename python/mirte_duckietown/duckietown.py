from ._topic import Subscriber


class Camera:
    """Camera API

    This class allows you to get access to useful camera data.
    """

    def __init__(
        self,
        subscriber=None,
        stop_line_threshold_height=0.75,
        tag_life=500,
    ):
        """Initialises a new camera

        Parameters:
            subscriber (Subscriber): The subscriber to use for fetching ROS topics. If
                None, a new subscriber will be created
            stop_line_threshold_height (float): The theshold where the stop
                line is considered to be visible
            tag_life (int): The number of milliseconds a tag will be remembered
                until it is considered expired
        """
        # Initialise subscriber
        if subscriber is None:
            self.__subscriber = Subscriber(tag_life=tag_life)
        else:
            self.__subscriber = subscriber

        # Stop line threshold height
        self.__stop_line_threshold_height = stop_line_threshold_height

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
            float: The height of the stop line in the camera image or None if
            the stop line is not visible. The height is normalised to [0.0, 1.0]
            where 0.0 is the top of the image and 1.0 is the bottom of the
            image.
        """
        stop_line = self.getStopLine()
        if stop_line is None or stop_line.direction.x_coord == 0:
            return None

        # Calculate y-intercept
        y_intercept = stop_line.origin.y_coord + (
            0.5 - stop_line.origin.x_coord
        ) * (stop_line.direction.y_coord / stop_line.direction.x_coord)

        # Clamp to [0.0, 1.0]
        return max(min(y_intercept, 1.0), 0.0)

    def seesStopLine(self):
        """Checks if the robot sees the stop line close enough

        Returns:
            bool: True if the robot sees the stop line close enough, False otherwise
        """
        stop_line_height = self.getStopLineHeight()
        if stop_line_height is None:
            return False
        # Check if stop line is close enough
        return stop_line_height >= self.__stop_line_threshold_height

    def getImage(self):
        """Gets the current camera image

        Returns:
            Image: Current image in OpenCV format, or None if no image is available
        """
        return self.__subscriber.getImage()

    def getAprilTags(self):
        """Gets the april tags from the camera

        Returns:
            list: List of AprilTag objects
        """
        return self.__subscriber.getAprilTags()


def createCamera():
    """Creates a Camera object

    Returns:
        Camera: The created Camera object
    """
    return Camera()
