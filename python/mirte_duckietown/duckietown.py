import threading

import rospy
from ._topic import Subscriber


class Camera:
    """Camera API

    This class allows you to get access to useful camera data.
    """

    def __init__(
        self,
        robot=None,
        subscriber=None,
        stop_line_threshold_height=0.75,
        tag_life=500,
    ):
        """Initialises a new camera

        Parameters:
            robot (Robot): The robot to use for controlling the robot.
                If None, you will not be able to control the robot automatically.
            subscriber (Subscriber): The subscriber to use for fetching ROS topics.
                If None, a new subscriber will be created.
            stop_line_threshold_height (float): The threshold where the stop
                line is considered to be visible
            tag_life (int): The number of milliseconds a tag will be remembered
                until it is considered expired
        """
        # Initialise subscriber
        if subscriber is None:
            self.__subscriber = Subscriber(tag_life=tag_life)
            self.__rospy_rate = rospy.Rate(30)
        else:
            self.__subscriber = subscriber

        # Initialise robot
        self.__robot = robot

        # Stop line threshold height
        self.__stop_line_threshold_height = stop_line_threshold_height

        # Don't yet follow the lane
        # self.__following = False

        # Start executing of the follower
        #print("starting execution...")

        # Run the follower in a separate thread
        #if self.__robot is not None:
		#	self.__thread = threading.Thread(target=self._follower)

    def getLines(self):
        """Gets line segments from the camera

        Returns:
            list[LineSegment]: List of LineSegment objects
        """
        return self.__subscriber.getLines()

    def getLane(self):
        """Gets the lane from the camera

        Returns:
            Lane: Lane
        """
        return self.__subscriber.getLane()

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
        y_intercept = stop_line.origin.y_coord + (0.5 - stop_line.origin.x_coord) * (
            stop_line.direction.y_coord / stop_line.direction.x_coord
        )

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

    def _follower(self):
        """Follows the lane using the camera"""
        while not self.__following:
            lane = self.__subscriber.getLane()
            if lane is not None:
                # Variables
                speed = 65
                turn_speed = 10
                turn_speed_corr = 5
                off_lane_threshold = 0.5
                off_lane_correction = 30

                # Calculate angle and correction
                angle = lane.centre_line.angle
                start = lane.centre_line.start
                if start < -off_lane_threshold:  # on the right, so turn left
                    angle -= off_lane_correction
                elif start > off_lane_threshold:  # on the left, so turn right
                    angle += off_lane_correction

                # calculate speed
                speed_left = speed
                speed_right = speed
                # Turn right when the angle is positive
                if angle > 10:
                    speed_left += turn_speed
                    speed_right -= turn_speed + turn_speed_corr
                # Turn left when the angle is negative
                elif angle < -10:
                    speed_left -= turn_speed + turn_speed_corr
                    speed_right += turn_speed

                # Set motor speeds
                self.__robot.setMotorSpeed("left", speed_left)
                self.__robot.setMotorSpeed("right", speed_right)
            self.__rospy_rate.sleep()

    def startFollowing(self):
        """Start following the lane using the camera, if the robot is initialized"""
        if self.__robot is None:
            return
		self.__thread = threading.Thread(target=self._follower)
		self.__following = True
		self.__thread.daemon = True
        self.__thread.start()

    def stopFollowing(self):
        """Stop following the lane using the camera, if the robot is initialized"""
        self.__following = False
		self.__thread.join()
        if self.__robot is not None:
            self.__robot.setMotorSpeed("left", 0)
            self.__robot.setMotorSpeed("right", 0)

    def getAprilTags(self):
        """Gets the april tags from the camera

        Returns:
            list[AprilTag]: List of AprilTag objects
        """
        return self.__subscriber.getAprilTags()

    def seesSign(self, sign):
        """Checks if the robot sees the given sign

        Parameters:
            sign (Sign): The sign to check for

        Returns:
            bool: True if the robot sees the sign, False otherwise
        """

        for tag in self.getAprilTags():
            if tag.toSign() == sign:
                return True
        return False

    def seesStreet(self, street_name: str):
        """Checks if the robot sees the given street

        Parameters:
            street_name (str): The street to check for

        Returns:
            bool: True if the robot sees the street, False otherwise
        """
        # Convert street name to uppercase and remove trailing dot
        street_name = street_name.upper().rstrip(".")
        # Check if street name is in the list of street names
        for tag in self.getAprilTags():
            if tag.getStreetName() == street_name:
                return True
        return False


def createCamera(robot=None):
    """Creates a Camera object

    Parameters:
        robot (Robot): The robot object to use for controlling the robot.

    Returns:
        Camera: The created Camera object
    """
    return Camera(robot)
