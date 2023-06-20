import math
import threading

import rospy
from mirte_msgs.msg import Lane as LaneROS
from mirte_msgs.srv import SetMotorSpeed
from simple_pid import PID


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
    following: bool = False
    __motors = {}
    __motor_services = {}
    __set_speed_func = None
    __pid = PID(0.5, 0.1, 0.05, setpoint=0)

    def __init__(self, set_motor_speed_func=None):
        if set_motor_speed_func is None:
            self.__default_init()
        else:
            self.__set_motor_speed = set_motor_speed_func

        threading.Thread(target=self.__follower).start()  # Run the follower in a separate thread
        try:
            rospy.init_node("lane_follower", anonymous=True)
        except rospy.exceptions.ROSException as e:
            pass
        rospy.Subscriber("lanes", LaneROS, self.ros_callback)

    def __default_init(self):
        # Copied from robot.py
        if rospy.has_param("/mirte/motor"):
            self.__motors = rospy.get_param("/mirte/motor")
            for motor in self.__motors:
                self.__motor_services[motor] = rospy.ServiceProxy(
                    '/mirte/set_' + self.__motors[motor]["name"] + '_speed',
                    SetMotorSpeed, persistent=True)
        self.__set_motor_speed = self.__default_set_motor_speed

    def __default_set_motor_speed(self, motor, value):
        motor = self.__motor_services[motor](value)
        return motor.status

    def ros_callback(self, data: LaneROS):
        self.current_lane = Lane(data)

    def get_lane(self):
        return self.current_lane

    def start_following(self):
        self.following = True

    def stop_following(self):
        self.following = False

    def __follower(self):
        while not rospy.is_shutdown():
            if self.following and self.current_lane is not None:
                speed = 65
                turn_speed = 10
                turn_speed_corr = 5

                angle = self.current_lane.centre_line.angle
                speed_left = speed
                speed_right = speed
                if angle > 10:
                    speed_left += turn_speed
                    speed_right -= (turn_speed + turn_speed_corr)
                elif angle < -10:
                    speed_left -= (turn_speed + turn_speed_corr)
                    speed_right += turn_speed
                self.__set_motor_speed('left', int(speed_left * 0.985))
                self.__set_motor_speed('right', speed_right)
            else:
                self.__set_motor_speed('left', 0)
                self.__set_motor_speed('right', 0)
            rospy.sleep(0.01)  # Prevent overloading ROS with messages


# Calculates an angle from a vector [x, y]
def calculate_radians(x, y):
    return math.atan2(y, x)


# Converts an angle in radians to degrees, where 0 degrees is straight up, and positive angles are clockwise
def convert_angle_to_degrees(angle):
    return math.degrees(angle) + 90


# Calculates the intercept of a line given by two points with the line y=1 (the bottom of the image)
def calculate_y1_intercept(x1, y1, x2, y2):
    if x1 == x2:
        return x1  # vertical line, so intercept will be the x coordinate
    a = (y2 - y1)/(x2 - x1)
    b = y1-a*x1
    return (1 - b)/a
