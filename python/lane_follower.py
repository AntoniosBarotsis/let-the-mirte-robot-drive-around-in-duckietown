import math

import rospy
from mirte_msgs.msg import LineSegmentList
from mirte_msgs.msg import Lane as LaneROS


class Line:
    # X coordinate of the line at the bottom of the image.
    # -1 is at the left of the image, 1 is at the right of the image.
    start: float = 0

    # Angle of the line in degrees.
    # 0 is pointing up, 90 is pointing right, -90 is pointing left.
    angle = 0

    # Angle of the line in radians.
    # Uses standard mathematical notation (0 is pointing right).
    radians = 0

    def __init__(self, line):
        self.radians = calculate_radians(line.direction.x, line.direction.y)
        self.angle = convert_angle_to_degrees(self.radians)
        self.start = calculate_y1_intercept(line.origin.x, line.origin.y, line.origin.x + line.direction.x, line.origin.y + line.direction.y)

    def __str__(self):
        return f"Line: Start: {self.start}, Angle: {self.angle}"


class Lane:
    left_line: Line = None
    centre_line: Line = None
    right_line: Line = None

    def __init__(self, lane):
        self.left_line = Line(lane.left)
        self.centre_line = Line(lane.centre)
        self.right_line = Line(lane.right)

    def __str__(self):
        return f"Lane Left: {self.left_line}, Centre: {self.centre_line}, Right: {self.right_line}"


class Follower:
    current_lane: Lane = None

    def __init__(self):
        rospy.init_node("lane_follower", anonymous=True)
        rospy.Subscriber("lanes", LaneROS, self.ros_callback)

    def ros_callback(self, data: LaneROS):
        self.current_lane = Lane(data)

    def get_lane(self):
        return self.current_lane

    def follow_lane(self):
        print("todo")


# Calculates an angle from a vector [x, y]
def calculate_radians(x, y):
    return math.atan2(y, x)


# Converts an angle in radians to degrees, where 0 degrees is straight up, and positive angles are clockwise
def convert_angle_to_degrees(angle):
    return -(math.degrees(angle) - 90)


# Calculates the intercept of a line given by two points with the line y=1 (the bottom of the image)
def calculate_y1_intercept(x1, y1, x2, y2):
    if x1 == x2:
        return x1  # vertical line, so intercept will be the x coordinate
    a = (y2 - y1)/(x2 - x1)
    b = y1-a*x1
    return (1 - b)/a
