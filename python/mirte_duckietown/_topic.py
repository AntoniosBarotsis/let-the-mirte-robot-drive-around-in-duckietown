from __future__ import annotations
from datetime import datetime
import signal
import sys
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray as AprilTagMsg
from mirte_duckietown_msgs.msg import (
    LineSegmentList as LineSegmentMsg,
    Line as LineMsg,
    ObstacleList as ObstacleMsg,
    Lane as LaneMsg,
)
from ._common import LineSegment, Line, Lane, AprilTag, TagDatabase, Obstacle


class Subscriber:
    """ROS subscriber

    This class allows you to acces ROS topics. The getters
    are just wrapper calling ROS topics.
    """

    __line_segments: list[LineSegment] = []
    __stop_line: Line = None
    __current_image: Image = None
    __bridge: CvBridge = None
    __april_tags: list[AprilTag] = []
    __tag_life: int
    __lane: Lane = None
    __obstacles = []

    def __init__(self, tag_life=500):
        self.__tag_life = tag_life

        # Callback for line segments
        def lineSegmentCb(data: LineSegmentMsg):
            self.__line_segments = []
            for segment in data.segments:
                self.__line_segments.append(LineSegment.fromMessage(segment))

        # Callback for stop line
        def stopLineCb(data: LineMsg):
            self.__stop_line = Line.fromMessage(data)

        def laneCb(data: LaneMsg):
            self.__lane = Lane.fromMessage(data)

        # Callback for april tags
        def aprilTagCb(data: AprilTagMsg):
            new_tags = []

            # Remove expired tags
            for tag in self.__april_tags:
                if not tag.hasExpired(self.__tag_life):
                    new_tags.append(tag)

            # Add new tags
            for detection in data.detections:
                tag = AprilTag(detection.id[0], datetime.now())
                if tag not in new_tags:
                    new_tags.append(tag)

            # Update tags
            self.__april_tags = new_tags

        # Callback for obstacles
        def obstacleCb(data: ObstacleMsg):
            self.__obstacles = []
            for obstacle in data.obstacles:
                self.__obstacles.append(Obstacle.fromMessage(obstacle))

        # Initialise node
        try:
            print("starting rospy...")
            rospy.init_node("camera", anonymous=True)
        except rospy.exceptions.ROSException:
            print("rospy is aleady running!")

        # Initialise subscribers

        rospy.Subscriber("line_segments", LineSegmentMsg, lineSegmentCb)
        rospy.Subscriber("stop_line", LineMsg, stopLineCb)
        rospy.Subscriber("lanes", LaneMsg, laneCb)
        rospy.Subscriber("tag_detections", AprilTagMsg, aprilTagCb)
        rospy.Subscriber("/obstacles", ObstacleMsg, obstacleCb)

        # Load tag database
        TagDatabase()

        # Shutdown handler
        def shutdownHandler(signum, frame):
            rospy.signal_shutdown(f"signal: {signum}\nframe: {frame}")
            print("stopping execution...")
            sys.exit()

        # Register shutdown handler
        signal.signal(signal.SIGINT, shutdownHandler)

    def getLines(self):
        """Gets line segments from ROS

        Returns:
            list[LineSegment]: List of LineSegment objects
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
        if self.__bridge is None:
            self.__bridge = CvBridge()

            # Callback for image
            def imageCb(data: Image):
                self.__current_image = self.__bridge.imgmsg_to_cv2(
                    data, desired_encoding="passthrough"
                )

            rospy.Subscriber("webcam/image_raw", Image, imageCb)

        return self.__current_image

    def getLane(self):
        """Gets the current lane from ROS

        Returns:
            Lane: Current lane
        """
        return self.__lane

    def getAprilTags(self):
        """Gets the april tags from ROS

        Returns:
            list[AprilTag]: List of AprilTag objects
        """
        return self.__april_tags

    def getObstacles(self):
        """Gets the current obstacles from ROS

        Returns:
            list: List of Obstacle objects
        """
        return self.__obstacles
