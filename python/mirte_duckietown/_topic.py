import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mirte_msgs.msg import (
    LineSegmentList as LineSegmentMsg,
    Line as LineMsg,
    Lane as LaneMsg,
)
from ._common import LineSegment, Line, Lane


class Subscriber:
    """ROS subscriber

    This class allows you to acces ROS topics. The getters
    are just wrapper calling ROS topics.
    """

    def __init__(self):
        # Initialise private fields
        self.__line_segments = []
        self.__stop_line = None
        self.__current_image = None
        self.__bridge = CvBridge()
        self.__lane = None

        # Callback for line segments
        def lineSegmentCb(data: LineSegmentMsg):
            self.__line_segments = []
            for segment in data.segments:
                self.__line_segments.append(LineSegment.fromMessage(segment))

        # Callback for stop line
        def stopLineCb(data: LineMsg):
            self.__stop_line = Line.fromMessage(data)

        # Callback for image
        def imageCb(data: Image):
            self.__current_image = self.__bridge.imgmsg_to_cv2(
                data, desired_encoding="passthrough"
            )

        def laneCb(data: LaneMsg):
            self.__lane = Lane.fromMessage(data)


        # Initialise node and subscriptions
        rospy.init_node("camera", anonymous=True)
        rospy.Subscriber("line_segments", LineSegmentMsg, lineSegmentCb)
        rospy.Subscriber("stop_line", LineMsg, stopLineCb)
        rospy.Subscriber("webcam/image_raw", Image, imageCb)
        rospy.Subscriber("lanes", LaneMsg, laneCb)

    def getLines(self):
        """Gets line segments from ROS

        Returns:
            list: List of LineSegment objects
        """
        return self.__line_segments

    def getStopLine(self):
        """Gets the stop line from ROS

        Returns:
            Line: Stop line
        """
        return self.__stop_line

    def getImage(self):
        """Gets the current image from ROS

        Returns:
            Image: Current image
        """
        return self.__current_image

    def getLane(self):
        """Gets the current lane from ROS

        Returns:
            Lane: Current lane
        """
        return self.__lane
