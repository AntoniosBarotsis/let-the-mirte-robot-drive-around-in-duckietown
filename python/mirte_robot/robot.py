import rospy
from mirte_msgs.srv import SetMotorSpeed


class Robot:
    """Robot API

    This class allows you to control the robot. The setters are just wrappers
    calling ROS services.
    """

    __motors = {}
    __motor_services = {}

    def __init__(self):
        if rospy.has_param("/mirte/motor"):
            self.__motors = rospy.get_param("/mirte/motor")
            for motor in self.__motors:
                self.__motor_services[motor] = rospy.ServiceProxy(
                    "/mirte/set_" + self.__motors[motor]["name"] + "_speed",
                    SetMotorSpeed,
                    persistent=True,
                )

    def setMotorSpeed(self, motor, value):
        """Sets the speed of a motor

        Parameters:
            motor (str): The name of the motor
            value (int): The speed of the motor
        """
        motor = self.__motor_services[motor](value)
        return motor.status


def createRobot():
    """Creates a Robot object

    Returns:
        Robot: A Robot object
    """
    return Robot()
