import rospy
from sensor_msgs.msg import Image
from mirte_msgs.msg import (
    LineSegmentList as LineSegmentMsg,
    Line as LineMsg,
)
from line import LineSegment, Line
from cv_bridge import CvBridge


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
        def line_segment_cb(data: LineSegmentMsg):
            self.__line_segments = []
            for segment in data.segments:
                self.__line_segments.append(LineSegment.fromMessage(segment))

        # Callback for stop line
        def stop_line_cb(data: LineMsg):
            self.__stop_line = Line.fromMessage(data)

        # Initialise image
        self.__current_image = None
        self.__bridge = CvBridge()

        # Callback for image
        def cv_image_cb(data: Image):
            self.__current_image = data

        # Initialise node and subscribers
        rospy.init_node("camera", anonymous=True)
        rospy.Subscriber("line_segments", LineSegmentMsg, line_segment_cb)
        rospy.Subscriber("stop_line", LineMsg, stop_line_cb)
        rospy.Subscriber("webcam/image_raw", Image, cv_image_cb)

        # Listen at 30 Hz
        self.rate = rospy.Rate(30)

    def sleep(self):
        """Sleep for one cycle of the camera

        Note:
            It is recommended to call this function at the end of your loop,
            because it there is no point in running the loop faster than the
            camera can update.
        """
        self.rate.sleep()

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

    def getImage(self):
        """Gets the current camera image

        Returns:
            Image: Current image in OpenCV format, or None if no image is available
        """
        if self.__current_image is None:
            return None
        return self.__bridge.imgmsg_to_cv2(self.__current_image, desired_encoding="passthrough")
