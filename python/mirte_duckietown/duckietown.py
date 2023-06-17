import rospy
from mirte_msgs.msg import (
    LineSegmentList as LineSegmentMsg,
    Line as LineMsg,
)
from .common import LineSegment, Line


class Camera:
    """Camera API

    This class allows you to access the camera data from the robot. The getters
    are just wrapper calling ROS topics.
    """

    def __init__(self):
        # Initialise line segments
        self.__line_segments = []
        self.__stop_line = None

        # Callback for line segments
        def lineSegmentCb(data: LineSegmentMsg):
            self.__line_segments = []
            for segment in data.segments:
                self.__line_segments.append(LineSegment.fromMessage(segment))

        # Callback for stop line
        def stopLineCb(data: LineMsg):
            self.__stop_line = Line.fromMessage(data)

        # Initialise node and subscribers
        rospy.init_node("camera", anonymous=True)
        rospy.Subscriber("line_segments", LineSegmentMsg, lineSegmentCb)
        rospy.Subscriber("stop_line", LineMsg, stopLineCb)

    def getLines(self):
        """Gets line segments from the camera

        Returns:
            list: List of LineSegment objects
        """
        return self.__line_segments

    def getStopLine(self):
        """Gets the stop line from the camera

        Returns:
            Line: Stop line
        """
        return self.__stop_line

    def stopLineDist(self):
        """Gets the high of the stop line in the camera image. The closer the
        robot is to the stop line, the lower the value.

        Returns:
            float: The height [0,1] of the stop line in the camera image
        """
        if self.__stop_line is None or self.__stop_line.direction.x_coord == 0:
            return None

        x_intercept = 0.5
        y_intercept = self.__stop_line.origin.y_coord + (
            x_intercept - self.__stop_line.origin.x_coord
        ) * (
            self.__stop_line.direction.y_coord
            / self.__stop_line.direction.x_coord
        )

        return 1.0 - y_intercept

    def stopLine(self):
        """Checks if the robot is in front of the stop line

        Returns:
            bool: True if the robot is in front of the stop line
        """
        distance = self.stopLineDist()
        if distance is None:
            return False
        return distance < 0.4


def createCamera():
    """Creates a Camera object

    Returns:
        Camera: The created Camera object
    """
    return Camera()
