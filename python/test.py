import rospy
from mirte_msgs.srv import SetMotorSpeed


class Follower:
    __motors = {}
    __motor_services = {}

    def __init__(self):
        print()
        # Copied from robot.py
        if rospy.has_param("/mirte/motor"):
            self.__motors = rospy.get_param("/mirte/motor")
            for motor in self.__motors:
                self.__motor_services[motor] = rospy.ServiceProxy(
                    "/mirte/set_" + self.__motors[motor]["name"] + "_speed",
                    SetMotorSpeed,
                    persistent=True,
                )

    def set_motor_speed(self, motor, value):
        motor = self.__motor_services[motor](value)
        return motor.status
